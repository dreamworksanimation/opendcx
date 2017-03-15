///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016 DreamWorks Animation LLC. 
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// *       Redistributions of source code must retain the above
//         copyright notice, this list of conditions and the following
//         disclaimer.
// *       Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials
//         provided with the distribution.
// *       Neither the name of DreamWorks Animation nor the names of its
//         contributors may be used to endorse or promote products
//         derived from this software without specific prior written
//         permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
///
/// @file DcxDeepTransform.cpp


#include "DcxDeepTransform.h"

#include <algorithm> // for std::sort in some compilers
#include <assert.h>

//#define DCX_DEBUG_RESAMPLING 1
//#define DCX_DEBUG_COLLAPSING 1

#if defined(DCX_DEBUG_RESAMPLING) || defined(DCX_DEBUG_COLLAPSING)
#  include <assert.h>
#  define SAMPLER_X 100
#  define SAMPLER_Y 100
#  define SAMPLER_Z 0
#  define SAMPLINGXY(A, B) (A==SAMPLER_X && B==SAMPLER_Y)
#  define SAMPLINGXYZ(A, B, C) (A==SAMPLER_X && B==SAMPLER_Y && C==SAMPLER_Z)
#endif


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

static int SSAMPLING_MAX = int(powf(2.0f, DeepTransform::SS_RATE_MAX-1));


//-------------------------------------------------------------------
// TODO: 2D vec mults are missing from ImathMatrix...
inline
IMATH_NAMESPACE::V2f
XY_x_M44(float x, float y, const IMATH_NAMESPACE::M44f& m)
{
    return IMATH_NAMESPACE::V2f(x * m[0][0] + y * m[1][0] + m[3][0],
                                x * m[0][1] + y * m[1][1] + m[3][1]);
}
//-------------------------------------------------------------------


//-------------------------------------------------------------------------------


DeepTransform::DeepTransform (SuperSamplingMode super_sampling,
                              FilterMode filter_mode) :
    m_ss_mode(SS_RATE_1),
    m_filter_mode(filter_mode),
    m_updated(false)
{
    setSuperSamplingMode(super_sampling, true/*force*/);
    m_matrix.makeIdentity();  // << may be redundant, think M44 ctor does this
    m_imatrix.makeIdentity(); // << may be redundant, think M44 ctor does this
}

DeepTransform::DeepTransform (const IMATH_NAMESPACE::M44f& m,
                              SuperSamplingMode super_sampling,
                              FilterMode filter_mode) :
    m_matrix(m),
    m_ss_mode(SS_RATE_1),
    m_filter_mode(filter_mode),
    m_updated(false)
{
    setSuperSamplingMode(super_sampling, true/*force*/);
}

/*virtual*/
DeepTransform::~DeepTransform ()
{
    //
}


//-------------------------------------------------------------------------------


/*virtual*/
uint32_t
DeepTransform::superSamplingRate () const
{
    return std::min(int(powf(2.0f, m_ss_mode)), SSAMPLING_MAX);
}

/*virtual*/
uint32_t
DeepTransform::superSamplingWidth () const
{
    return superSamplingRate()*SpMask8::width;
}

/*virtual*/
void
DeepTransform::setSuperSamplingMode (SuperSamplingMode f,
                                     bool force)
{
    SuperSamplingMode new_mode = std::max(SS_RATE_1, std::min(f, DeepTransform::SuperSamplingMode(SS_RATE_MAX-1)));
    //std::cout << "DeepTransform(" << this << ")::setSuperSamplingMode(): m_ss_mode=" << m_ss_mode << ", new_mode=" << new_mode;
    if (force || m_ss_mode != new_mode)
    {
        ILMTHREAD_NAMESPACE::Lock lock(m_mutex);
        if (force || m_ss_mode != new_mode)
        {
            m_ss_mode = new_mode;
            fillSuperSamplingTables(superSamplingWidth()/*ss_width*/, 1/*num_tables*/);
        }
    }
}

//
// Default implementation fills one table with uniform distribution subpixel
// offsets.
// TODO: implement a stochastic version
//

/*virtual*/
void
DeepTransform::fillSuperSamplingTables (uint32_t ss_width,
                                        uint32_t num_tables)
{
    //std::cout << "DeepTransform(" << this << ")::fillSuperSamplingOffsetTables(): ss_width=" << ss_width << ", num_tables=" << num_tables << std::endl;

    // Simple linear distribution:
    const float ss_scale  = 1.0f/float(ss_width);
    const float ss_center = 0.5f/float(ss_width);

    if (num_tables < 1)
        num_tables = 1;
    m_sp_offset_tables.clear();
    m_sp_offset_tables.resize(num_tables);
    for (uint32_t i=0; i < num_tables; ++i)
    {
        SampleOffsetTable& ss_table = m_sp_offset_tables[i];
        ss_table.resize(ss_width*ss_width);

        uint32_t sample_index = 0;
        for (uint32_t ssY=0; ssY < ss_width; ++ssY)
        {
            for (uint32_t ssX=0; ssX < ss_width; ++ssX)
            {
                SampleOffset& ss_offset = ss_table[sample_index++];
                ss_offset.dx = ss_center + float(ssX)*ss_scale;
                ss_offset.dy = ss_center + float(ssY)*ss_scale;
                ss_offset.wt = 1.0f; // unused atm
                //std::cout << " [" << ss_offset.dx << " " << ss_offset.dy << "]";
            }
            //std::cout << std::endl;
        }
    }
}

//
// Default implementation always returns table 0 for all pixel coords.
//

/*virtual*/
const DeepTransform::SampleOffsetTable&
DeepTransform::getSuperSamplingTable (int /*pixel_x*/,
                                      int /*pixel_y*/) const
{
    const size_t table_index = 0; // do something pseudo-random here

#ifdef DEBUG
    assert(table_index < m_sp_offset_tables.size());
#endif
    return m_sp_offset_tables[table_index];
}


//-------------------------------------------------------------------------------


const IMATH_NAMESPACE::M44f&
DeepTransform::imatrix()
{
    if (!m_updated)
    {
        ILMTHREAD_NAMESPACE::Lock lock(m_mutex);
        if (!m_updated)
        {
            m_imatrix = m_matrix.inverse();
            m_updated = true;
        }
    }
    return m_imatrix;
}

/*static*/
void
DeepTransform::transform (const IMATH_NAMESPACE::Box2i& in,
                          const IMATH_NAMESPACE::M44f& m,
                          IMATH_NAMESPACE::Box2i& out)
{
    // Add +1 to r/t so we're transforming pixel region corners:
    const IMATH_NAMESPACE::V2f p0 = XY_x_M44(float(in.min.x  ), float(in.min.y  ), m); // bottom-left
    const IMATH_NAMESPACE::V2f p1 = XY_x_M44(float(in.max.x+1), float(in.min.y  ), m); // bottom-right
    const IMATH_NAMESPACE::V2f p2 = XY_x_M44(float(in.max.x+1), float(in.max.y+1), m); // top-right
    const IMATH_NAMESPACE::V2f p3 = XY_x_M44(float(in.min.x  ), float(in.max.y+1), m); // top-left
    // TODO: enable pad or add as passed-in option?
    const float rounding_pad = -0.001f;//0.0f;//0.5f;
    // Find the x->r && y->t range:
    out.min.x = int(floorf(std::min(p0.x, std::min(p1.x, std::min(p2.x, p3.x))) - rounding_pad)); // X
    out.min.y = int(floorf(std::min(p0.y, std::min(p1.y, std::min(p2.y, p3.y))) - rounding_pad)); // Y
    out.max.x = int( ceilf(std::max(p0.x, std::max(p1.x, std::max(p2.x, p3.x))) + rounding_pad)); // R
    out.max.y = int( ceilf(std::max(p0.y, std::max(p1.y, std::max(p2.y, p3.y))) + rounding_pad)); // T
    // ...and back to inclusive:
    if (out.max.x > out.min.x)
        --out.max.x;
    if (out.max.y > out.min.y)
        --out.max.y;
}


//-------------------------------------------------------------------------------


//
//  Sample all input deep pixels contributing to output deep pixel outX/outY.
//

/*virtual*/
void
DeepTransform::sample (int outX, int outY,
                       const DeepTile& deep_in_tile,
                       const ChannelSet& sample_channels,
                       Dcx::DeepPixel& deep_out_pixel)
{
    deep_out_pixel.clear();
    deep_out_pixel.setChannels(sample_channels);
#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    const bool debug_enable = SAMPLINGXY(outX, outY);
#endif
    // No filtering, find nearest input pixel and copy it:
    if (m_filter_mode == FILTER_NEAREST)
    {
        // Back-project center of pixel:
        float inXf, inYf;
        backTransform(float(outX)+0.5f,  float(outY)+0.5f, inXf, inYf, 0/*clamp_to*/);
        const int inX = int(floorf(inXf)+0.01f);
        const int inY = int(floorf(inYf)+0.01f);
        // Skip if projected outside input bbox:
        if (inX < deep_in_tile.minX() || inX > deep_in_tile.maxX() || 
            inY < deep_in_tile.minY() || inY > deep_in_tile.maxY())
            return;

        deep_in_tile.getDeepPixel(inX, inY, deep_out_pixel);

        return;
    }

    // Back-project output pixel corners into input tile.
    // backTransform expects an inclusive-coordinate bbox, so we don't add +1 to r/t here:
    const IMATH_NAMESPACE::Box2i in_bbox = backTransform(IMATH_NAMESPACE::Box2i(IMATH_NAMESPACE::V2i(outX, outY),
                                                                                IMATH_NAMESPACE::V2i(outX, outY)), 0/*clamp_to*/);

#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    if (debug_enable) {
        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "DeepTransform::sample(" << outX << ", " << outY << ") yUp=" << deep_in_tile.yAxisUp();
        //std::cout << ", dataWindow" << deep_in_tile.dataWindow() << std::endl;
        std::cout << "  transform:" << std::endl << std::fixed << imatrix();
        std::cout << "  sample input rectangle=" << in_bbox;
        std::cout << "=[ " << (in_bbox.max.x - in_bbox.min.x + 1) << "x" << (in_bbox.max.y - in_bbox.min.y + 1) << " ]";
    }
#endif

    // Bail if it transformed outside input bbox:
    if (in_bbox.max.x < deep_in_tile.minX() || in_bbox.min.x > deep_in_tile.maxX() || 
        in_bbox.max.y < deep_in_tile.minY() || in_bbox.min.y > deep_in_tile.maxY())
#ifdef DCX_DEBUG_RESAMPLING
    {
        if (debug_enable) std::cout << "...sample rectangle outside input tile bbox, bail." << std::endl;
        return;
    }
#else
        return;
#endif

    Dcx::DeepPixel deep_in_pixel(deep_in_tile.channels());

    std::vector<SegmentRef> sample_segments;
    sample_segments.reserve(100);

    // Iterate through input pixel range building a list of segments that may contribute
    // to the output pixel.
    // This list is z-sorted to make it easier to combine partial coverages.
    for (int tY=in_bbox.min.y; tY <= in_bbox.max.y; ++tY)
    {
        for (int tX=in_bbox.min.x; tX <= in_bbox.max.x; ++tX)
        {
            // Get the deep pixel from the deep tile:
            deep_in_tile.getDeepPixel(tX, tY, deep_in_pixel);
            const uint32_t nSegments = deep_in_pixel.size();
#ifdef DCX_DEBUG_RESAMPLING
            if (debug_enable) {
                std::cout << "  --------------------------------------------------------------------" << std::endl;
                std::cout << "  in[" << tX << " " << tY << "] segments=" << nSegments << std::endl;
            }
#endif

            if (nSegments == 0)
                continue;
#ifdef DEBUG
            assert(nSegments < 10000); // just in case...
#endif

            for (uint32_t segment_index=0; segment_index < nSegments; ++segment_index)
            {
                const DeepSegment& in_segment = deep_in_pixel.getSegment(segment_index);
                if (isinf(in_segment.Zf) || isinf(in_segment.Zb))
                    continue;
                sample_segments.push_back(SegmentRef(tX, tY, segment_index, in_segment.Zf, in_segment.spMask()));
            }

        } // tX loop

    } // tY loop

    // Sample the set of input deep pixels into this output pixel:
    std::sort(sample_segments.begin(), sample_segments.end());
    sampleSegments(deep_in_tile, sample_segments, sample_channels, outX, outY, deep_out_pixel);

}


//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------


//
//  Transform a single input deep pixel and resample it to an output deep tile.
//

/*virtual*/
void DeepTransform::transformPixel (const DeepTile& deep_in_tile,
                                    int inX, int inY,
                                    DeepTile& deep_out_tile)
{
#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    const bool debug_enable = SAMPLINGXY(inX, inY);
#endif
    // Only sample channels shared by both tiles:
    ChannelSet sample_channels(deep_out_tile.channels());
    sample_channels &= deep_in_tile.channels();

    // Forward-project input pixel corners into output tile.
    // transform expects an inclusive-coordinate bbox, so we don't add +1 to r/t here:
    const IMATH_NAMESPACE::Box2i out_bbox = transform(IMATH_NAMESPACE::Box2i(IMATH_NAMESPACE::V2i(inX, inY),
                                                                             IMATH_NAMESPACE::V2i(inX, inY)), 0/*clamp_to*/);

#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    if (debug_enable) {
        std::cout << "----------------------------- transformPixel ---------------------------------" << std::endl;
        std::cout << "DeepTransform::transformPixel(" << inX << ", " << inY << ")";
        std::cout << "  transform:" << std::endl << std::fixed << matrix();
        std::cout << "  covers output rectangle=" << out_bbox;
        std::cout << "=[ " << (out_bbox.max.x - out_bbox.min.x + 1) << "x" << (out_bbox.max.y - out_bbox.min.y + 1) << " ]";
    }
#endif

    // Bail if it transformed outside output bbox:
    if (out_bbox.max.x < deep_out_tile.minX() || out_bbox.min.x > deep_out_tile.maxX() || 
        out_bbox.max.y < deep_out_tile.minY() || out_bbox.min.y > deep_out_tile.maxY())
#ifdef DCX_DEBUG_RESAMPLING
    {
        if (debug_enable) std::cout << "...sample rectangle outside output tile bbox, bail." << std::endl;
        return;
    }
#else
        return;
#endif

    // Get the input deep pixel and build the list of input segment references:
    DeepPixel deep_pixel(sample_channels); // reused
    deep_pixel.reserve(10);
    deep_in_tile.getDeepPixel(inX, inY, deep_pixel);
    const uint32_t nSegments = deep_pixel.size();
#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    if (debug_enable) {
        std::cout << ", nSegments=" << nSegments << ":" << std::endl;
    }
#endif
    if (nSegments == 0)
        return; // No segments, bail

    if (m_filter_mode == FILTER_NEAREST)
    {
        // No filtering - copy input pixel to output pixels:
        for (int outY=out_bbox.min.y; outY <= out_bbox.max.y; ++outY)
        {
            for (int outX=out_bbox.min.x; outX <= out_bbox.max.x; ++outX)
            {
                // Back-project center of output pixel:
                float inXf, inYf;
                backTransform(float(outX)+0.5f,  float(outY)+0.5f, inXf, inYf, 0/*clamp_to*/);
                const int inx = int(floorf(inXf)+0.01f);
                const int iny = int(floorf(inYf)+0.01f);
#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
                if (debug_enable)
                    std::cout << "    (" << outX << ", " << outY << "): inX=" << inx << ", inY=" << iny << std::endl;
#endif
                if (inx == inX && iny == inY)
                    deep_out_tile.setDeepPixel(outX, outY, deep_pixel);
            }
        }

    }
    else
    {
        // Filtering - build the list of segment references to pass to sample method:
#ifdef DEBUG
        assert(nSegments < 10000); // just in case...
#endif
        std::vector<SegmentRef> sample_segments;
        sample_segments.reserve(nSegments);
        for (uint32_t i=0; i < nSegments; ++i)
        {
            const DeepSegment& in_segment = deep_pixel.getSegment(i);
            if (isinf(in_segment.Zf) || isinf(in_segment.Zb))
                continue;
            sample_segments.push_back(SegmentRef(inX, inY, i, in_segment.Zf, in_segment.spMask()));
        }
        std::sort(sample_segments.begin(), sample_segments.end());

        // Distribute the deep pixel into all the output pixels it overlaps:
        for (int outY=out_bbox.min.y; outY <= out_bbox.max.y; ++outY)
        {
            for (int outX=out_bbox.min.x; outX <= out_bbox.max.x; ++outX)
            {
                deep_out_tile.getDeepPixel(outX, outY, deep_pixel);
                sampleSegments(deep_in_tile, sample_segments, sample_channels, outX, outY, deep_pixel);
                deep_out_tile.setDeepPixel(outX, outY, deep_pixel);
            }
        }

    }

}


//
//  Transform all the pixels of a tile.
//

/*virtual*/
void
DeepTransform::transformTile (const DeepTile& deep_in_tile,
                              DeepTile& deep_out_tile)
{
    // Only sample channels shared by both tiles:
    ChannelSet xform_channels(deep_out_tile.channels());
    xform_channels &= deep_in_tile.channels();

    DeepPixel deep_pixel(xform_channels); // reused
    deep_pixel.reserve(10);

    for (int outY=deep_out_tile.minY(); outY <= deep_out_tile.maxY(); ++outY)
    {
        for (int outX=deep_out_tile.minX(); outX <= deep_out_tile.maxX(); ++outX)
        {
            Dcx::DeepTransform::sample(outX, outY, deep_in_tile, xform_channels, deep_pixel);
            deep_out_tile.setDeepPixel(outX, outY, deep_pixel);
        }
    }

}


//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------

// Sample deep pixels contributing to output deep pixel outX/outY from
// a set of preselected input segments, with subpixel-mask resampling.
//
// Note: deep_out_pixel is not cleared by this method, allowing multiple
// input DeepPixels to be combined into it.
//
// This method is called by the other sample/transform methods and
// does the actual work of resampling the segments.
//

void
DeepTransform::sampleSegments (const DeepTile& deep_in_tile,
                               const std::vector<SegmentRef>& sample_segments,
                               const ChannelSet& sample_channels,
                               int outX, int outY,
                               Dcx::DeepPixel& deep_out_pixel)
{
    deep_out_pixel.setXY(outX, outY);

#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    const bool debug_enable = SAMPLINGXY(outX, outY);
    if (debug_enable)
        deep_out_pixel.setXY(-100000,-100000); // debug coord
#endif

    const uint32_t nSegments = sample_segments.size();
    if (nSegments == 0)
        return; // No valid segments to combine

    //=============================================================
    // Do partial subpixel-coverage filtering
    //=============================================================

    const float depth_threshold = 0.001f;//EPSILONf;
    const float color_threshold = 0.001f;//EPSILONf;

    const int ss_rate     = superSamplingRate();
    const int ss_rate_sqr = ss_rate*ss_rate;
    const int ss_width    = superSamplingWidth();
    const SampleOffsetTable& ss_offsets = getSuperSamplingTable(outX, outY);

    const float maskWf = float(SpMask8::width);
    const float outXf = float(outX);
    const float outYf = float(outY);
    float inXf, inYf;
#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
    if (debug_enable) {
        std::cout << "  ------------------------ sampleSegments ---------------------------" << std::endl;
        std::cout << "  DeepTransform::sampleSegments(): nSegments=" << nSegments;
        std::cout << ", ss_rate=" << ss_rate << ", ss_rate_sqr=" << ss_rate_sqr;
        std::cout << ", ss_width=" << ssW << std::endl;
    }
#endif

    Dcx::DeepPixel deep_in_pixel(sample_channels);
    //const float bin_scale = DeepFlags::maxSpCoverageScale / float(ss_rate_sqr);
    std::vector<char> bin_hits(SpMask8::numBits);
    std::vector<char> ss_weight_counts(ss_rate_sqr);
    std::vector<SpMask8> ss_weight_masks(ss_rate_sqr);
    SpMask8 out_mask, out_opaque_mask, out_partial_mask;
    DeepSegment out_segment;
    Pixelf out_pixel;

    // Loop through the segments adding input segments to the output list while
    // combining opaque and partial contribution segments as much as possible
    // to reduce output segment count:
    for (uint32_t ref_index=0; ref_index < nSegments; ++ref_index)
    {
#ifdef DEBUG
        assert(ref_index < sample_segments.size());
#endif
        const SegmentRef& seg_ref = sample_segments[ref_index];
#if defined DCX_DEBUG_COLLAPSING || defined DCX_DEBUG_RESAMPLING
        if (debug_enable) {
            std::cout << "  ===============================================================================================" << std::endl;
            std::cout << "  " << ref_index << " z=" << seg_ref.Zf << " [" << seg_ref.x << " " << seg_ref.y << "] segment=" << seg_ref.segment << std::endl;
        }
#endif

        out_mask = out_opaque_mask = SpMask8::zeroCoverage;
        memset(&bin_hits[0], 0, SpMask8::numBits);

        // Sample subpixel locations at a potentially higher rate than 8x8,
        // building up the bin-hit counts and accumulated masks:
        for (int spSupOutY=(ss_width-1); spSupOutY >= 0; --spSupOutY)
        {
#ifdef DCX_DEBUG_RESAMPLING
            if (debug_enable) std::cout << "  ";
#endif
            const int spOutY = spSupOutY / ss_rate;
            for (int spSupOutX=0; spSupOutX < ss_width; ++spSupOutX)
            {
                const SampleOffset& ss_offset = ss_offsets[spSupOutY*ss_width + spSupOutX];

                const int spOutX = spSupOutX / ss_rate;
                // Back-project input subpixel coordinate into output space:
                backTransform(outXf + ss_offset.dx, outYf + ss_offset.dy, inXf, inYf);
                const float inX_flr = floorf(inXf);
                const float inY_flr = floorf(inYf);
                const int inX = int(inX_flr);
                const int inY = int(inY_flr);
                // Did it project inside the current input pixel:
                if (inX == seg_ref.x && inY == seg_ref.y)
                {
                    // Sample location in input spmask:
                    const int spInX = int(floorf((inXf - inX_flr)*maskWf));
                    const int spInY = int(floorf((inYf - inY_flr)*maskWf));

                    // Get input spmask bit:
                    const SpMask8 in_sp = SpMask8(1ull) << (spInY*SpMask8::width + spInX);
                    if (seg_ref.spmask & in_sp)
                    {
                        // Increment hit-count for this output bin:
                        const int out_bin = spOutY*SpMask8::width + spOutX;
#ifdef DCX_DEBUG_RESAMPLING
                        assert(out_bin < (int)SpMask8::numBits);
#endif
                        ++bin_hits[out_bin];
                        const SpMask8 out_sp = SpMask8(1ull) << out_bin;
                        out_mask |= out_sp;
                        if (bin_hits[out_bin] >= ss_rate_sqr)
                            out_opaque_mask |= out_sp;
                    }
#ifdef DCX_DEBUG_RESAMPLING
                    //if (debug_enable) printf(" %02d", spInY*SpMask8::width + spInX);
                    if (debug_enable) printf(" %02d", spOutY*SpMask8::width + spOutX);
#endif
                }
#ifdef DCX_DEBUG_RESAMPLING
                else
                    if (debug_enable) std::cout << "  .";
#endif

            } // spInX loop
#ifdef DCX_DEBUG_RESAMPLING
            if (debug_enable) std::cout << std::endl;
#endif

        } // spInY loop
#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable) {
            std::cout << "  bin hit counts:" << std::endl;
            for (int yy=SpMask8::height-1; yy>=0; --yy) {
                std::cout << "  ";
                for (int xx=0; xx < SpMask8::width; ++xx) {
                    const int hits = bin_hits[yy*SpMask8::width + xx];
                    if (hits > 0)
                        printf(" %02d", hits);
                    else
                        std::cout << "  .";
                }
                std::cout << std::endl;
            }
        }
#endif

        // Skip this segment if it doesn't overlap any output bins:
        if (out_mask == SpMask8::zeroCoverage)
#ifdef DCX_DEBUG_COLLAPSING
        {
            if (debug_enable) std::cout << "     no output overlap, skip segment " << seg_ref.segment << std::endl;
            continue;
        }
#else
            continue;
#endif

        // Partial mask is the enabled bits that are not opaque:
        out_partial_mask = (out_mask & ~out_opaque_mask);

        deep_in_tile.getDeepPixel(seg_ref.x, seg_ref.y, deep_in_pixel);
        deep_in_pixel.setXY(seg_ref.x, seg_ref.y);
#ifdef DEBUG
        assert(seg_ref.segment < deep_in_pixel.size());
#endif
        const DeepSegment& in_segment = deep_in_pixel.getSegment(seg_ref.segment);
        const Pixelf& in_pixel = deep_in_pixel.getSegmentPixel(in_segment);

#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable) {
            std::cout << "  ---------------------------------------------------------------------------" << std::endl;
            if (0) {
                std::cout << "   out_mask:" << std::endl;
                out_mask.printPattern(std::cout, "     ");
                std::cout << "   out_opaque_mask:" << std::endl;
                out_opaque_mask.printPattern(std::cout, "     ");
                std::cout << "   out_partial_mask:" << std::endl;
                out_partial_mask.printPattern(std::cout, "     ");
            }
            std::cout << "    " << seg_ref.segment << " Zf=" << in_segment.Zf << ", Zb=" << in_segment.Zb;
            std::cout << " flags=[";
            in_segment.printFlags(std::cout);
            std::cout << "] chans" << in_pixel;
            std::cout << std::endl;
        }
#endif

        // Write the opaque segment if the mask is non-zero:
        if (out_opaque_mask != SpMask8::zeroCoverage)
        {
#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) std::cout << "    OPAQUE:" << std::endl;
#endif

            out_segment = in_segment;
            out_segment.metadata.spmask = out_opaque_mask;
            deep_out_pixel.appendOrCombineSegment(out_segment, in_pixel, depth_threshold, color_threshold);
        }

        // Write partially-transparent segment if segment is included in accumulated partial mask:
        // The transparent mask is the bins that are on but not opaque:
        if (out_partial_mask != SpMask8::zeroCoverage)
        {
            // Determine weights for each bin, then output the dominant weight, or
            // separate segments with the separate weights separated by subpixel bits:
            memset(&ss_weight_counts[0], 0, ss_rate_sqr);
            memset(&ss_weight_masks[0],  0, ss_rate_sqr*sizeof(SpMask8));

            for (int sp_bin=0; sp_bin < SpMask8::numBits; ++sp_bin)
            {
                const int hits = bin_hits[sp_bin];
                if (hits == 0 || hits >= ss_rate_sqr)
                     continue; // shouldn't happen...
                // Increment weight count for ss bin:
                ++ss_weight_counts[hits];
                ss_weight_masks[hits] |= (SpMask8(1ull) << sp_bin);
            }

            // Output separate segments for each weight bin, building a unique
            // subpixel mask for each:
#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) {
                std::cout << "    PARTIAL: enabled bins" << std::endl;
                std::cout << "    [" << std::endl;
            }
#endif
            for (int bin_index=1; bin_index < ss_rate_sqr; ++bin_index)
            {
                if (ss_weight_counts[bin_index] > 0)
                {
                    out_segment = in_segment;
                    out_segment.metadata.spmask = ss_weight_masks[bin_index];
                    // Set new partial subpixel-coverage weight and
                    // weight the output pixel channels by it:
                    const float spweight = float(bin_index) / float(ss_rate_sqr);
                    out_segment.setSpCoverageWeight(spweight);
                    out_pixel = deep_in_pixel.getSegmentPixel(out_segment);
                    out_pixel *= spweight;

                    deep_out_pixel.appendOrCombineSegment(out_segment,
                                                          out_pixel,
                                                          depth_threshold,
                                                          color_threshold);
#if 0//def DCX_DEBUG_COLLAPSING
                    if (debug_enable)
                        deep_out_pixel.printInfo(std::cout, "        out=", 10/*padding*/, true/*show_mask*/);
#endif
                }

            }
#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) std::cout << "    ]" << std::endl;
#endif

        } // has partial subpixel-coverage

#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable)
            deep_out_pixel.printInfo(std::cout, "  out_pixel=", 3/*padding*/, true/*show_mask*/);
#endif

    } // SegmentRefs loop


#if defined DCX_DEBUG_RESAMPLING && !defined DCX_DEBUG_COLLAPSING
    if (debug_enable)
        deep_out_pixel.printInfo(std::cout, "  out_pixel=", 3/*padding*/, true/*show_mask*/);
#endif
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
