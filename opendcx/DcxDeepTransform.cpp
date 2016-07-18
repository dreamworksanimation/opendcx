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

#include <assert.h>

#define SSAMPLING_MAX 8

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

// TODO: move this somewhere common or use another lock class...
struct GlobalLock
{
    pthread_mutex_t m_mutex;

    GlobalLock () { pthread_mutex_init(&m_mutex, NULL); }
    ~GlobalLock () { pthread_mutex_destroy(&m_mutex); }

    void lock () { pthread_mutex_lock(&m_mutex); }
    void unlock () { pthread_mutex_unlock(&m_mutex); }
    void spinlock () {
        while (pthread_mutex_trylock(&m_mutex))
            sched_yield();
    }
};
static GlobalLock       g_access_lock;


//-------------------------------------------------------------------------------


DeepTransform::DeepTransform (int super_sampling,
                              FilterMode filter_mode) :
    m_updated(false),
    m_zTranslate(0.0f),
    m_zScale(1.0f),
    m_filter_mode(filter_mode),
    m_ss_factor(std::max(1, std::min(super_sampling, 32)))
{
    m_matrix.makeIdentity();  // << may be redundant, think M44 ctor does this
    m_imatrix.makeIdentity(); // << may be redundant, think M44 ctor does this
}

DeepTransform::DeepTransform (const IMATH_NAMESPACE::M44f& m,
                              int super_sampling,
                              FilterMode filter_mode) :
    m_matrix(m),
    m_updated(false),
    m_zTranslate(0.0f),
    m_zScale(1.0f),
    m_filter_mode(filter_mode),
    m_ss_factor(std::max(1, std::min(super_sampling, 32)))
{
    //
}

/*virtual*/
DeepTransform::~DeepTransform ()
{
    //
}

//-------------------------------------------------------------------------------


const IMATH_NAMESPACE::M44f&
DeepTransform::imatrix()
{
    if (!m_updated)
    {
        g_access_lock.lock();
        if (!m_updated)
        {
            m_imatrix = m_matrix.inverse();
            m_updated = true;
        }
        g_access_lock.unlock();
    }
    return m_imatrix;
}


//-------------------------------------------------------------------------------


/*static*/
void
DeepTransform::transform (const IMATH_NAMESPACE::Box2i& in,
                          const IMATH_NAMESPACE::M44f& m,
                          IMATH_NAMESPACE::Box2i& out)
{
    // Change r/t to exclusive:
    const IMATH_NAMESPACE::V3f p0 = IMATH_NAMESPACE::V3f(float(in.min.x  ), float(in.min.y  ), 0.0f) * m; // bottom-left
    const IMATH_NAMESPACE::V3f p1 = IMATH_NAMESPACE::V3f(float(in.max.x+1), float(in.min.y  ), 0.0f) * m; // bottom-right
    const IMATH_NAMESPACE::V3f p2 = IMATH_NAMESPACE::V3f(float(in.max.x+1), float(in.max.y+1), 0.0f) * m; // top-right
    const IMATH_NAMESPACE::V3f p3 = IMATH_NAMESPACE::V3f(float(in.min.x  ), float(in.max.y+1), 0.0f) * m; // top-left
    // TODO: enable pad or add as passed-in option?
    const float rounding_pad = 0.0f;//0.5f;
    // Find the x->r && y->t range:
    out.min.x = int(floorf(std::min(p0.x, std::min(p1.x, std::min(p2.x, p3.x))) - rounding_pad)); // X
    out.min.y = int(floorf(std::min(p0.y, std::min(p1.y, std::min(p2.y, p3.y))) - rounding_pad)); // Y
    out.max.x = int( ceilf(std::max(p0.x, std::max(p1.x, std::max(p2.x, p3.x))) + rounding_pad)); // R
    out.max.y = int( ceilf(std::max(p0.y, std::max(p1.y, std::max(p2.y, p3.y))) + rounding_pad)); // T
    //std::cout << "  p3" << p3 << ", i3[" << out.min.x << " " << out.max.y << "] p2" << p2 << ", i2[" << out.max.x << " " << out.max.y << "]" << std::endl;
    //std::cout << "  p0" << p0 << ", io[" << out.min.x << " " << out.min.y << "] p1" << p1 << ", i1[" << out.max.x << " " << out.min.y << "]" << std::endl;
    // Back to inclusive:
    if (out.max.x > out.min.x)
        --out.max.x;
    if (out.max.y > out.min.y)
        --out.max.y;
    //std::cout << "  out" << out << " [" << (out.max.x - out.min.x + 1) << " " << (out.max.y - out.min.y + 1) << "]" << std::endl;
}

//-------------------------------------------------------------------------------

/*virtual*/
void
DeepTransform::sample (int outX,
                       int outY,
                       const DeepTile& deep_in_tile,
                       Dcx::DeepPixel& out_pixel
#ifdef DCX_DEBUG_TRANSFORM
                       , bool debug
#endif
                       )
{
    out_pixel.clear();

    // No filtering, find nearest input pixel and copy it:
    if (m_filter_mode == FILTER_NEAREST)
    {
        // Back project center of pixel:
        float finX, finY;
        backTransform(float(outX)+0.5f,  float(outY)+0.5f, finX, finY, 0/*clamp_to*/);
        const int inX = int(floorf(finX)+0.01f);
        const int inY = int(floorf(finY)+0.01f);

        if (inX < deep_in_tile.x() || inX > deep_in_tile.r() || 
            inY < deep_in_tile.y() || inY > deep_in_tile.t())
            return; // Skip if projected outside input bbox

        deep_in_tile.getDeepPixel(inX, inY, out_pixel);

        return;
    }

    // Back-project output pixel corners into input tile.
    // backTransform expects an inclusive bbox, so we don't add one to r/t here:
    const IMATH_NAMESPACE::Box2i in = backTransform(IMATH_NAMESPACE::Box2i(IMATH_NAMESPACE::V2i(outX, outY),
                                                                           IMATH_NAMESPACE::V2i(outX, outY)), 0/*clamp_to*/);

#ifdef DCX_DEBUG_TRANSFORM
    if (debug) {
        std::cout << "----------------------------------------------------------------------------------" << std::endl;
        std::cout << "DeepTransform::sample(" << outX << ", " << outY << ") yUp=" << deep_in_tile.tileYup();
//        std::cout << ", dataWindow" << deep_in_tile.dataWindow() << std::endl;
        //std::cout << "  transform:" << std::endl << std::fixed << imatrix();
//        std::cout << "  sample input rectangle=" << in;
        std::cout << " [" << (in.max.x - in.min.x + 1) << " " << (in.max.y - in.min.y + 1) << "]";
    }
#endif

    // Bail if it transformed outside input bbox:
    if (in.min.x < deep_in_tile.x() || in.max.x > deep_in_tile.r() || 
        in.min.y < deep_in_tile.y() || in.max.y > deep_in_tile.t())
#ifdef DCX_DEBUG_TRANSFORM
    {
        if (debug) std::cout << "...sample rectangle outside input tile bbox, bail." << std::endl;
        return;
    }
#else
        return;
#endif

    Dcx::DeepPixel in_pixel(deep_in_tile.channels());

    //=============================================================
    // Partial-transparency filtering
    //=============================================================

    const int   ss_factor = std::min(std::max(m_ss_factor, 1), SSAMPLING_MAX);
    const int   ss_factor_sqr = ss_factor*ss_factor;
    const int   superW = (int)SpMask8::width*ss_factor; // 32
    const float ifsuperW = 1.0f/float(superW);
    const float ss_factor_center = 0.5f/float(superW);
    const float fmaskW = float(SpMask8::width);
    const float foutX = float(outX);
    const float foutY = float(outY);
    float finX, finY;
#ifdef DCX_DEBUG_TRANSFORM
    if (debug) std::cout << ", ss_factor=" << ss_factor << ", ss_factor_sqr=" << ss_factor_sqr << std::endl;
#endif

    char    bin_hits[SpMask8::numBits];
    std::vector<char> ss_weight_counts(ss_factor_sqr);   //char    ss_weight_counts[ss_factor_sqr];
    std::vector<SpMask8> ss_weight_masks(ss_factor_sqr); //SpMask8 ss_weight_masks[ss_factor_sqr];
    SpMask8 out_mask, out_opaque_mask, out_transp_mask;

    // Iterate through input pixel range, finding segments that contribute to the output pixel,
    // and resampling the masks:
    for (int tY=in.min.y; tY <= in.max.y; ++tY)
    {
        for (int tX=in.min.x; tX <= in.max.x; ++tX)
        {
            // Get the deep pixel from the deep tile:
            deep_in_tile.getDeepPixel(tX, tY, in_pixel
#ifdef DCX_DEBUG_FLATTENER
                                      , debug
#endif
                                      );
            const size_t nSegments = in_pixel.size();
#ifdef DCX_DEBUG_TRANSFORM
            if (debug) {
                std::cout << "  --------------------------------------------------------------------" << std::endl;
                std::cout << "  in[" << tX << " " << tY << "] segments=" << nSegments << std::endl;
            }
#endif

            if (nSegments == 0)
                continue;
            assert(nSegments < 10000); // just in case...

            // There's input subpixel masks, so resample them:
            for (size_t segment=0; segment < nSegments; ++segment)
            {
                // Get input segment:
                const DeepSegment& in_segment = in_pixel.getSegment(segment);
                const SpMask8 in_mask = (!in_segment.zeroCoverage())?in_segment.spMask():SpMask8::fullCoverage;

#ifdef DCX_DEBUG_TRANSFORM
                if (debug) {
                    std::cout << "   -----------------------------" << std::endl;
                    std::cout << "   " << segment << " Zf=" << in_segment.Zf << ", Zb=" << in_segment.Zb << std::endl;
                    std::cout << "   mask:" << std::endl;
                    in_mask.printPattern(std::cout, "   ");
                }
#endif
                out_mask = out_opaque_mask = SpMask8::zeroCoverage;
                memset(bin_hits, 0, SpMask8::numBits);

                // Sample subpixel locations at a potentially higher rate and stochastically(TODO)
                // than 8x8, building up the bin-hit counts and accumulated masks:
                for (int spSupOutY=(superW-1); spSupOutY >= 0; --spSupOutY)
                {
#ifdef DCX_DEBUG_TRANSFORM
                    if (debug) std::cout << "  ";
#endif
                    const int spOutY = spSupOutY / ss_factor;
                    for (int spSupOutX=0; spSupOutX < superW; ++spSupOutX)
                    {
                        const int spOutX = spSupOutX / ss_factor;
                        // Back-project input subpixel coordinate into output space:
                        // (TODO: can this be a dPdx/dpDy interpolation routine instead?)
                        // (TODO: jitter this stochastically too!)
                        backTransform(foutX + ss_factor_center + float(spSupOutX)*ifsuperW,
                                      foutY + ss_factor_center + float(spSupOutY)*ifsuperW,
                                      finX, finY);
                        const float flr_inX = floorf(finX);
                        const float flr_inY = floorf(finY);
                        const int inX = int(flr_inX);
                        const int inY = int(flr_inY);
                        // Skip if it projects outside the current output pixel:
                        if (inX != tX || inY != tY)
#ifdef DCX_DEBUG_TRANSFORM
                        {
                            if (debug) std::cout << " [   ]  ";
                            continue;
                        }
#else
                            continue;
#endif

                        // Sample location in input spmask:
                        const int spInX = int(floorf((finX - flr_inX)*fmaskW));
                        const int spInY = int(floorf((finY - flr_inY)*fmaskW));
#ifdef DCX_DEBUG_TRANSFORM
                        if (debug) std::cout << " [" << spInX << " " << spInY << "]";
#endif

                        // Get input spmask bit:
                        const SpMask8 in_sp = SpMask8(1ull) << (spInY*SpMask8::width + spInX);
                        if (in_mask & in_sp)
                        {
                            // Increment hit-count for this output bin:
                            const int out_bin = spOutY*SpMask8::width + spOutX;
                            assert(out_bin < (int)SpMask8::numBits);
                            const SpMask8 out_sp = SpMask8(1ull) << out_bin;
                            out_mask |= out_sp;
                            ++bin_hits[out_bin];
                            if (bin_hits[out_bin] >= ss_factor_sqr)
                                out_opaque_mask |= out_sp;
#ifdef DCX_DEBUG_TRANSFORM
                            if (debug) std::cout << "=" << std::dec << (int)bin_hits[out_bin];
#endif
                        }
#ifdef DCX_DEBUG_TRANSFORM
                        else
                        {
                            if (debug) std::cout << "  ";
                        }
#endif

                    } // spInX loop
#ifdef DCX_DEBUG_TRANSFORM
                    if (debug) std::cout << std::endl;
#endif

                } // spInY loop

                // Skip this segment if it doesn't overlap any output bins:
                if (out_mask == SpMask8::zeroCoverage)
#ifdef DCX_DEBUG_TRANSFORM
                {
                    if (debug) std::cout << "     no output overlap, skip segment " << segment << std::endl;
                    continue;
                }
#else
                    continue;
#endif

                // Write the opaque sample if the mask is non-zero:
                if (out_opaque_mask != SpMask8::zeroCoverage) {
                    DeepSegment& out_segment = out_pixel.getSegment(out_pixel.append(in_pixel, segment));
                    // Update output metadata:
                    out_segment.metadata.spmask = out_opaque_mask;
                    out_segment.metadata.flags &= ~DEEP_PARTIAL_BIN_COVERAGE;
                }

                // Write partially-transparent segment if segment is included in accumulated transp mask:
                // The transparent mask is the bins that are on but not opaque:
                out_transp_mask = (out_mask & ~out_opaque_mask);
#if 0//def DCX_DEBUG_TRANSFORM
                if (debug) {
                    std::cout << "   out_mask" << std::endl;
                    out_mask.printPattern(std::cout, "   ");
                    std::cout << "   out_opaque_mask" << std::endl;
                    out_opaque_mask.printPattern(std::cout, "   ");
                    std::cout << "   out_transp_mask" << std::endl;
                    out_transp_mask.printPattern(std::cout, "   ");
                }
#endif
                if (out_transp_mask != SpMask8::zeroCoverage)
                {
                    // Determine weights for each bin, then output the dominant weight, or
                    // separate segments with the separate weights separated by subpixel bits:
                    memset(&ss_weight_counts[0], 0, ss_factor_sqr);
                    memset(&ss_weight_masks[0], 0, ss_factor_sqr*sizeof(SpMask8));

                    float average_weight = 0.0f;
                    int transp_hits = 0;
                    SpMask8 count_mask(1ull);
                    for (int sp_bin=0; sp_bin < SpMask8::numBits; ++sp_bin, count_mask <<= 1)
                    {
                        const int nHits = bin_hits[sp_bin];
                        if (nHits == 0 || nHits >= ss_factor_sqr)
                             continue;
                        // Increment weight count for ss bin:
                        ++ss_weight_counts[nHits];
                        ss_weight_masks[nHits] |= (SpMask8(1ull) << sp_bin);
                        //
                        const float bin_weight = float(nHits) / float(ss_factor_sqr);
                        average_weight += bin_weight;
                        ++transp_hits;
                    }
                    //
                    if (1)
                    {
                        // Output separate segments for each weight bin, building a unique
                        // subpixel mask for each:
                        for (int i=1; i < ss_factor_sqr; ++i)
                        {
                            const int weight_count = ss_weight_counts[i];
                            if (weight_count == 0)
                                continue;
                            DeepSegment& out_segment = out_pixel.getSegment(out_pixel.append(in_pixel, segment));
                            // Weight output channels:
                            out_pixel.getSegmentPixel(out_segment) *= float(i) / float(ss_factor_sqr);
                            // Update output metadata:
                            out_segment.metadata.spmask = ss_weight_masks[i];
                            out_segment.metadata.flags |= DEEP_PARTIAL_BIN_COVERAGE;
                        }
                    }
                    else
                    {
                        // Output only the dominant (just the average for now...) weight:
                        // TODO: change this to some other weighting...?
                        DeepSegment& out_segment = out_pixel.getSegment(out_pixel.append(in_pixel, segment));
                        // Weight output channels:
                        out_pixel.getSegmentPixel(out_segment) *= (average_weight / float(transp_hits));
                        // Update output metadata:
                        out_segment.metadata.spmask = out_transp_mask;
                        out_segment.metadata.flags |= DEEP_PARTIAL_BIN_COVERAGE;
                    }

                }

            } // segments loop

        } // tX loop

    } // tY loop

}


/*virtual*/
void
DeepTransform::transformTile (const DeepTile& in_tile,
                              DeepTile& out_tile
#ifdef DCX_DEBUG_TRANSFORM
                              , bool debug
#endif
                              )
{
    ChannelSet do_channels(out_tile.channels());
    do_channels &= in_tile.channels(); // Only process shared tile channels

    DeepPixel out_pixel(out_tile.channels());
    out_pixel.reserve(10);

    for (int outY=out_tile.y(); outY <= out_tile.t(); ++outY)
    {
        for (int outX=out_tile.x(); outX <= out_tile.r(); ++outX)
        {
            Dcx::DeepTransform::sample(outX, outY, in_tile, out_pixel
#ifdef DCX_DEBUG_TRANSFORM
                                        , debug/*debug*/
#endif
                                        );
#ifdef DCX_DEBUG_TRANSFORM
            if (debug) {
                std::cout << "out[" << outX << " " << outY << "]" << std::endl;
                out_pixel.printInfo(std::cout, "out_pixel=", 4/*padding*/);
            }
#endif
            out_tile.setDeepPixel(outX, outY, out_pixel);
        }
    }

}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
