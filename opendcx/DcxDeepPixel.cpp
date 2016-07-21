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
/// @file DcxDeepPixel.cpp

#include "DcxDeepPixel.h"

#include <algorithm> // for std::sort in some compilers


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

// Uncomment this to get lots of info from flattener:
//#define DCX_DEBUG_FLATTENER 1
#ifdef DCX_DEBUG_FLATTENER
#  include <assert.h>
#endif


// For now we're using a fixed number of steps:
#define SEGMENT_SAMPLE_STEPS 5
//#define SEGMENT_SAMPLE_STEPS 15


// Maxium absorbance is what we consider an opaque surface.
// For this we are using an alpha of 0.999999999999
// So absorbance = -log( 1.0 - alpha )
// So absorbance = more or less 12.0
#define MAX_ABSORBANCE  12.0

// Clamp 0...1
template <class T>
inline
T CLAMP(T v)
{
    return std::max((T)0, std::min((T)1, v));
}

//----------------------------------------------------------------------------------------

/*static*/ const SpMask8 SpMask8::zeroCoverage(SpMask8::allBitsOff);
/*static*/ const SpMask8 SpMask8::fullCoverage(SpMask8::allBitsOn);

/*friend*/
std::ostream&
operator << (std::ostream& os,
             const SpMask8& mask)
{
    os << "0x";
    const std::ios_base::fmtflags flags = os.flags();
    const char fill = os.fill();
    const int w = os.width();
    os.fill('0');
    os.width(16);
    os << std::hex << mask.value();
    os.flags(flags);
    os.fill(fill);
    os.width(w);
    return os;
}

//----------------------------------------------------------------------------------------

void
DeepMetadata::printFlags (std::ostream& os) const
{
    if (flags == DEEP_EMPTY_FLAG)
    {
        os << "none";
        return;
    }
    if (flags & Dcx::DEEP_LINEAR_INTERP_SAMPLE)
        os << "Linear";
    else
        os << "Log";
    if (flags & Dcx::DEEP_MATTE_OBJECT_SAMPLE)
        os << ",Matte";
    if (flags & Dcx::DEEP_ADDITIVE_SAMPLE)
        os << ",Additive";
    if (flags & Dcx::DEEP_PARTIAL_BIN_COVERAGE)
        os << ",Partial-Bin-Coverage";
}

//----------------------------------------------------------------------------------------

//
// Prints segment info and metadata, but no channel data.
//

void
DeepSegment::printInfo (std::ostream& os) const
{
    const std::streamsize prec = os.precision();
    os.precision(8);
    os << "Zf=" << Zf << ", Zb=" << Zb << ", flags=("; printFlags(os); os << ")";
    os << ", spmask=" << metadata.spmask;
    os.precision(prec);
}

//
// Print a nicely formatted list of sample info including flags and subpixel mask patterns.
//

void
DeepPixel::printInfo (std::ostream& os,
                      const char* prefix,
                      int padding,
                      bool show_mask)
{
    if (prefix && prefix[0])
        os << prefix;
    os << "{";
    if (m_segments.size() > 0)
    {
        const std::ios_base::fmtflags flags = os.flags();
        const std::streamsize prec = os.precision();
        this->sort(true/*force*/); // Get global info up to date
        os << " overlaps=" << m_overlaps << " full_coverage=" << allFullCoverage();
        os << " ANDmask=" << m_accum_and_mask << " ORmask=" << m_accum_or_mask;
        os << " ANDflags=0x" << std::hex << m_accum_and_flags << " ORflags=0x" << m_accum_or_flags << std::dec;
        os << std::endl;
        padding = std::max(0, std::min((int)padding, 1024));
        std::vector<char> spaces(padding*2+1, ' ');
        spaces[spaces.size()-1] = 0;
        for (uint32_t i=0; i < m_segments.size(); ++i) {
            const DeepSegment& segment = m_segments[i];
            os.precision(8);
            os << &spaces[padding] << i << ": Zf=" << std::fixed << segment.Zf << " Zb=" << segment.Zb;
            //
            os << " flags=["; segment.printFlags(os); os << "]";
            //
            if (!show_mask)
                os << " mask=" << segment.spMask();
            //
            const Pixelf& color = getSegmentPixel(i);
            os << " chans=[";
            os.precision(6);
            foreach_channel(z, m_channels)
                std::cout << " " << *z << "=" << std::fixed << color[*z];
            os << " ]" << std::endl;
            if (show_mask)
                segment.spMask().printPattern(os, &spaces[0]);
        }
        os.flags(flags);
        os.precision(prec);
    }
    else
    {
        os << " empty ";
    }
    os << "}" << std::endl;
}

//----------------------------------------------------------------------------------------

//
// Outputs a raw list of values suitable for parsing.
//

/*friend*/
std::ostream&
operator << (std::ostream& os,
             const DeepSegment& ds)
{
    os << "[Zf=" << ds.Zf << ", Zb=" << ds.Zb << ", flags=" << ds.metadata.flags;
    os << ", spmask=" << std::hex << ds.metadata.spmask << std::dec << ", coverage=" << ds.getCoverage() << "]";
    return os;
}

//
// Outputs a raw list of samples and channel contents that's suitable for parsing.
//

std::ostream&
operator << (std::ostream& os,
             DeepPixel& dp)
{
    dp.sort(true/*force*/); // Get global info up to date
    os << "{";
    if (dp.size() > 0)
    {
        for (uint32_t i=0; i < dp.size(); ++i)
        {
            if (i > 0)
                os << " ";
            os << dp[i].Zf << ":" << dp[i].Zb << "[";
            foreach_channel(z, dp.channels())
                os << " " << dp.getChannel(i, *z);
            os << " ]";
        }
    }
    os << "}";
    return os;
}

//
//  Empty the segment list and clear shared values, except for channel set.
//

void
DeepPixel::clear ()
{
    m_segments.clear();
    m_pixels.clear();

    m_sorted = m_overlaps = false;
    m_accum_or_mask  = m_accum_and_mask  = SpMask8::zeroCoverage;
    m_accum_or_flags = m_accum_and_flags = DEEP_EMPTY_FLAG;
}


//
//  Add a DeepSegment to the end of the list.
//

size_t
DeepPixel::append (const DeepSegment& bs,
                   const Pixelf& bp)
{
    const size_t new_segment = m_segments.size();
    const size_t new_pixel   = m_pixels.size();

    if (m_segments.capacity() < m_segments.size()+1)
        m_segments.reserve(m_segments.size() + (size_t)int(float(m_segments.size())/1.5f));
    if (m_pixels.capacity() < m_pixels.size()+1)
        m_pixels.reserve(m_pixels.size() + (size_t)int(float(m_pixels.size())/1.5f));
    m_segments.push_back(bs);
    m_pixels.push_back(bp);

    m_pixels[new_pixel].channels  = m_channels;
    m_segments[new_segment].index = (int)new_pixel;

    m_sorted = m_overlaps = false;
    m_accum_or_mask  = m_accum_and_mask  = SpMask8::zeroCoverage;
    m_accum_or_flags = m_accum_and_flags = DEEP_EMPTY_FLAG;

    return new_segment;
}


//
//  Add an empty DeepSegment to the end of the list, returning its index.
//

size_t
DeepPixel::append ()
{
    return this->append(DeepSegment(), Pixelf(m_channels));
}


//
//  Add a DeepSegment to the end of the list.
//

size_t
DeepPixel::append (const DeepSegment& bs)
{
    return this->append(bs, Pixelf(m_channels));
}


//
//  Copy one segment from a second DeepPixel.
//

size_t
DeepPixel::append (const DeepPixel& b,
                   size_t segment_index)
{
#ifdef DCX_DEBUG_FLATTENER
    assert(segment_index < b.m_segments.size());
#endif
    const DeepSegment& bs = b[segment_index];
    const Pixelf& bp = b.m_pixels[bs.index];
    return this->append(bs, bp);
}


//
//  Combine the segments of two DeepPixels.
//

void
DeepPixel::append (const DeepPixel& b)
{
    const size_t nCurrent = m_segments.size();
    // Clear any new channels that are being added by the new set
    // so we don't end up with junk left in the memory:
    foreach_channel(z, b.m_channels)
    {
        if (!m_channels.contains(*z))
        {
            for (size_t i=0; i < nCurrent; ++i)
                m_pixels[i][z] = 0.0f;
        }
    }
    m_channels += b.m_channels; // OR the channel masks together
    // Copy the segments:
    const size_t nAdded = b.m_segments.size();
    m_segments.reserve(m_segments.size() + nAdded);
    m_pixels.reserve(m_pixels.size() + nAdded);
    for (size_t i=0; i < nAdded; ++i)
    {
        m_segments.push_back(b.m_segments[i]);
        m_pixels.push_back(b.m_pixels[i]);
        m_segments[m_segments.size()-1].index = (int)(m_pixels.size()-1);
        m_pixels[m_pixels.size()-1].channels = m_channels;
    }
    m_sorted = m_overlaps = false;
    m_accum_or_mask  = m_accum_and_mask  = SpMask8::zeroCoverage;
    m_accum_or_flags = m_accum_and_flags = DEEP_EMPTY_FLAG;
}


//
//  Return the index of the DeepSegment nearest to Z and inside the distance
//  of Z + or - maxDistance. Return -1 if nothing found.
//

int
DeepPixel::nearestSegment (double Z,
                           double maxDistance)
{
#if 1
    // TODO: finish implementing this!
    return (Z < maxDistance); // placeholder to avoid compile warning!
#else
    if (m_segments.size() == 0)
        return -1;
    this->sort();
    DeepSegment ds;
    ds.Zf = Z;
    std::vector<DeepSegment >::const_iterator i = lower_bound(m_segments.begin(), m_segments.end(), ds);
    if (i == m_segments.end())
        i = m_segments.end()-1;
    printf("Z=%f, i=%d[%f]\n", Z, (int)(i - m_segments.begin()), i->Zf);
    return (int)(i - m_segments.begin());
#endif
}


//
//  Sort the segments.  If the sorted flag is true this returns quickly.
//  This also updates global overlap and coverage flags.
//

void
DeepPixel::sort (bool force)
{
    if (m_sorted && !force)
        return;
    m_overlaps = false;
    m_accum_or_mask   = m_accum_and_mask  = SpMask8::zeroCoverage;
    m_accum_and_flags = m_accum_and_flags = DEEP_EMPTY_FLAG;
    const size_t nSegments = m_segments.size();
    if (nSegments > 0)
    {
        // Sort the segments:
        std::sort(m_segments.begin(), m_segments.end());

        // Determine global overlap and coverage status:
        m_accum_and_mask  = SpMask8::fullCoverage;
        m_accum_and_flags = DEEP_ALL_FLAGS;
        float prev_Zf = -INFINITYf;
        float prev_Zb = -INFINITYf;
        for (size_t i=0; i < nSegments; ++i)
        {
            const DeepSegment& segment = m_segments[i];
            if (segment.Zf < prev_Zf || segment.Zb < prev_Zf ||
                segment.Zf < prev_Zb || segment.Zb < prev_Zb)
                m_overlaps = true;

            m_accum_or_mask   |= segment.spMask();
            m_accum_and_mask  &= segment.spMask();
            m_accum_or_flags  |= segment.flags();
            m_accum_and_flags &= segment.flags();

            prev_Zf = segment.Zf;
            prev_Zb = segment.Zb;
        }
    }
    m_sorted = true;
}


//
//  Check for overlaps between samples and return true if so.
//  If the spmask arg is not full coverage then determine overlap
//  for the specific subpixel mask.
//

bool
DeepPixel::hasOverlaps (const SpMask8& spmask,
                        bool force)
{
    const size_t nSegments = m_segments.size();
    if (nSegments == 0)
        return false;
    // Sorting calculates the global overlap:
    if (!m_sorted || force)
        this->sort(force);
    // If full coverage then we can return the global overlap indicator:
    if (spmask == SpMask8::fullCoverage || allFullCoverage() || isLegacyDeepPixel())
        return m_overlaps;
    // Determine the overlap status for the spmask:
    float prev_Zf = -INFINITYf;
    float prev_Zb = -INFINITYf;
    for (size_t i=0; i < nSegments; ++i)
    {
        const DeepSegment& segment = m_segments[i];
        if (segment.maskBitsEnabled(spmask))
        {
            if (segment.Zf < prev_Zf || segment.Zb < prev_Zf ||
                segment.Zf < prev_Zb || segment.Zb < prev_Zb)
                return true;
            prev_Zf = segment.Zf;
            prev_Zb = segment.Zb;
        }
    }
    return false;
}

bool
DeepPixel::allZeroCoverage ()
{
    sort(); // updates flags
    return (m_accum_or_mask == SpMask8::zeroCoverage);
}

bool
DeepPixel::allFullCoverage ()
{
    sort(); // updates flags
    return (m_accum_and_mask == SpMask8::fullCoverage);
}

bool
DeepPixel::allVolumetric ()
{
    sort(); // updates flags
    return (m_accum_and_flags & DEEP_LINEAR_INTERP_SAMPLE)==0;
}
bool
DeepPixel::anyVolumetric ()
{
    sort(); // updates flags
    return (m_accum_or_flags & DEEP_LINEAR_INTERP_SAMPLE)==0;
}

bool
DeepPixel::allHardSurface ()
{
    sort(); // updates flags
    return (m_accum_and_flags & DEEP_LINEAR_INTERP_SAMPLE)!=0;
}
bool
DeepPixel::anyHardSurface ()
{
    sort(); // updates flags
    return (m_accum_or_flags & DEEP_LINEAR_INTERP_SAMPLE)!=0;
}

bool
DeepPixel::allMatte ()
{
    sort(); // updates flags
    return (m_accum_and_flags & DEEP_MATTE_OBJECT_SAMPLE)!=0;
}
bool
DeepPixel::anyMatte ()
{
    sort(); // updates flags
    return (m_accum_or_flags & DEEP_MATTE_OBJECT_SAMPLE)!=0;
}

bool
DeepPixel::isLegacyDeepPixel ()
{
    sort(); // updates flags
    return (allZeroCoverage() && allVolumetric());
}


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

//
//  Flatten the DeepSegments down into the output Pixel using front-to-back UNDERs.
//

void
DeepPixel::flatten (const ChannelSet& out_channels,
                    Pixelf& out,
                    InterpolationMode interpolation)
{
    const bool has_overlaps = hasOverlaps();
#ifdef DCX_DEBUG_FLATTENER
    if (debug) {
        std::cout << "    DeepPixel::flatten() overlaps=" << m_overlaps << ", full_coverage=" << allFullCoverage();
        std::cout << ", legacy=" << isLegacyDeepPixel();
        std::cout << ", accum_and_mask=" << std::hex << m_accum_and_mask << ", accum_and_flags=" << m_accum_and_flags << std::dec;
    }
#endif

    // If full coverage or there's no spmasks than we just have to flatten once:
    if (allFullCoverage() || isLegacyDeepPixel())
    {
#ifdef DCX_DEBUG_FLATTENER
        if (debug)
            std::cout << ": FULL-COVERAGE: interpolation=" << interpolation << std::endl;
#endif
        if (!has_overlaps || interpolation == OPENDCX_INTERNAL_NAMESPACE::INTERP_OFF)
            // No overlaps, do a simple linear flatten:
            flattenNoOverlaps(out_channels, out, SpMask8::fullCoverage);
        else
            // Merge overlapping deep segments:
            flattenOverlapping(out_channels, out, SpMask8::fullCoverage, interpolation);
        return;
    }

    //===================================
    // SUBPIXEL LOOP
    //===================================
#ifdef DCX_DEBUG_FLATTENER
    if (debug) std::cout << " - SUBPIXEL-LOOP: interpolation=" << interpolation << std::endl;
#endif

    out.erase(out_channels);
    //out[Chan_Z     ] =  INFINITYf;
    out[Chan_ZFront] =  INFINITYf;
    out[Chan_ZBack ] = -INFINITYf;

    Pixelf flattened(out_channels);

#if 0
    // TODO: handle the Z accumulation with coverage weighting - i.e. pick Z from
    //    the sample with largest coverage.
    //    Finishing this code will likely produce more accurate flattened Z's
    std::vector<int> frontmost_coverage;
    frontmost_coverage.resize(this->size(), 0);
#endif

    // If nearest cutout Z is in front of non-cutout, output INF:
    float cutout_Z = INFINITYf;

    // Harcode for now:
    const size_t subpixels_x = 8;
    const size_t subpixels_y = 8;

    SpMask8 sp_mask(1ull);
    size_t count = 0;
    for (size_t sp_y=0; sp_y < subpixels_y; ++sp_y)
    {
        for (size_t sp_x=0; sp_x < subpixels_x; ++sp_x)
        {
            // Flatten just the segments that have their masks on for this subpixel:
            flattenSubpixels(out_channels, flattened, sp_mask, interpolation);
            // Add flattened subpixel color to accumulation pixel:
            out += flattened;
#if 1
            if (flattened[Chan_CutoutZ] < INFINITYf)
            {
                // Flattened pixel has cutout in front, don't min accum chans:
                cutout_Z = std::min(flattened[Chan_CutoutZ], cutout_Z);

            }
            else
            {
                //out[Chan_Z     ] = std::min(out[Chan_Z     ], flattened[Chan_Z     ]);
                out[Chan_ZFront] = std::min(out[Chan_ZFront], flattened[Chan_ZFront]);
                out[Chan_ZBack ] = std::min(out[Chan_ZBack ], flattened[Chan_ZBack ]);
            }
#else
            // TODO: handle the Z accumulation with coverage weighting
            frontmost_coverage[] = 
#endif
            //
            ++sp_mask;
            ++count;
        }
    }

    // If final cutout Z is in front of non-cutout Z, output INF:
    if (cutout_Z < out[Chan_ZFront])
        /*out[Chan_Z] = */out[Chan_ZFront] = out[Chan_ZBack] = INFINITYf;

    if (count > 1)
        out /= float(count);

#if 0
    // TODO: handle the Z accumulation with coverage weighting - i.e. pick Z from
    //    the sample with largest coverage.
    //    Finishing this code will likely produce more accurate flattened Z's
    // Find frontmost with most counts:
    int max_coverage = -1;
    for (size_t i=0; i < nSpans; ++i)
    {
        if (frontmost_coverage[i] > 0 &&
              frontmost_coverage[i] > max_coverage)
        {
            max_coverage = frontmost_coverage[i];
            Z = segment_list[i].Zf;
        }
    }
#endif

} // DeepPixel::flatten


//
//  Flatten the DeepSegments for all subpixels in spmask into the output Pixel using front-to-back UNDERs.
//

void
DeepPixel::flattenSubpixels (const ChannelSet& out_channels,
                             Pixelf& out,
                             const SpMask8& spmask,
                             InterpolationMode interpolation)
{
    // Must update the overlap state for each bin individually.  This returns
    // fast if global full coverage is on:
    const bool has_overlaps = hasOverlaps(spmask, false/*force*/);
#ifdef DCX_DEBUG_FLATTENER
    if (debug) {
        std::cout << "    DeepPixel::flattenSubpixels() interpolation=" << interpolation;
        std::cout << ", sp_mask=" << std::hex << spmask << std::dec << ", overlaps=" << has_overlaps;
        std::cout << ", accum_and_mask=" << std::hex << m_accum_and_mask << ", accum_and_flags=" << m_accum_and_flags << std::dec;
        std::cout << ", full_coverage=" << allFullCoverage() << std::endl;
    }
#endif

    // Are there overlaps...?:
    if (!has_overlaps || interpolation == OPENDCX_INTERNAL_NAMESPACE::INTERP_OFF)
        // No overlaps, do a simple linear flatten:
        flattenNoOverlaps(out_channels, out, spmask);
    else
        // Merge overlapping deep segments:
        flattenOverlapping(out_channels, out, spmask, interpolation);
}

//-------------------------------------------------------------------------------------

//
//  Flatten a list of sorted segments.
//

void
DeepPixel::flattenNoOverlaps (const ChannelSet& out_channels,
                              Pixelf& out,
                              const SpMask8& spmask)
{
    out.erase(out_channels);
    // Always fill in these output channels even though they may not be enabled
    // in the output pixel's channel set:
    //out[Chan_Z      ] =  INFINITYf;
    out[Chan_ZFront ] =  INFINITYf;
    out[Chan_ZBack  ] = -INFINITYf;
    out[Chan_CutoutA] = 0.0f;
    out[Chan_CutoutZ] = INFINITYf;

    const size_t nSegments = m_segments.size();
#ifdef DCX_DEBUG_FLATTENER
    if (debug) {
//        std::cout << "    DeepPixel::flattenNoOverlaps=" << out_channels << ", nSegments=" << nSegments;
        std::cout << ", sp_mask=" << std::hex << spmask << std::dec << std::endl;
    }
#endif
    if (nSegments == 0)
    {
        out[Chan_ZBack] = INFINITYf;
        return; // shouldn't happen...
    }

    // Get the channel set to composite:
    ChannelSet comp_channels(out_channels);
    comp_channels &= m_channels; // only bother compositing requested output channels
    comp_channels += Chan_A;     // but, always composite alpha even if not requested
    comp_channels -= Mask_Depth; // don't composite depth channels, we handle those separately
    //
    ChannelSet comp_channels_no_alpha(comp_channels);
    comp_channels_no_alpha -= Chan_A;
#ifdef DCX_DEBUG_FLATTENER
    ChannelSet comp_channels_with_cutout(comp_channels);
    comp_channels_with_cutout += Chan_CutoutA;
#endif

    // Is this deep pixel full of legacy (no spmask, no interpolation flag) samples?
    const bool useSpMasks = !isLegacyDeepPixel();

#ifdef DCX_DEBUG_FLATTENER
    if (debug) {
        std::cout << "      segments:" << std::endl;
        std::cout.precision(8);
        std::cout << std::fixed;
        for (size_t i=0; i < nSegments; ++i) {
            const DeepSegment& segment = m_segments[i];
            std::cout << "        " << i << ": Zf=" << segment.Zf << ", Zb=" << segment.Zb;
            std::cout << ", flags=("; segment.printFlags(std::cout); std::cout << "), mask=" << std::hex << segment.spMask() << std::dec;
            std::cout.precision(6);
            std::cout << ", [";
            foreach_channel(z, comp_channels)
                std::cout << " " << *z << "=" << std::fixed << getChannel(i, *z);
            std::cout << " ]" << std::endl;
        }
    }
#endif

    // Put them in front to back order then composite using UNDER or PLUS operations:
    this->sort();
    for (size_t i=0; i < nSegments; ++i)
    {
        const DeepSegment& segment = m_segments[i];
#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout << "     " << i << std::fixed << ": Zf=" << segment.Zf << " Zb=" << segment.Zb;
            std::cout << " flags=("; segment.printFlags(std::cout); std::cout << ")";
        }
#endif

        // Skip segment if not in active spmask:
        if (useSpMasks && !segment.maskBitsEnabled(spmask))
#ifdef DCX_DEBUG_FLATTENER
        {
            if (debug) std::cout << " - subpixel OFF: skip it." << std::endl;
            continue;
        }
#else
            continue;
#endif

#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout << " - subpixel ON: segment_thickness=" << (segment.Zb - segment.Zf) << std::endl;
            if (useSpMasks)
                segment.spMask().printPattern(std::cout, "        ");
        }
#endif

        const Pixelf& color = getSegmentPixel(i);

        // UNDER or ADD each color channel:
        if (segment.isMatte())
        {
            // Matte object, color chans are black so just under alpha:
            if (segment.isAdditive() || segment.hasPartialSubpixelBinCoverage())
                out[Chan_A] += color[Chan_A]; // ADD
            else
                out[Chan_A] += color[Chan_A]*(1.0f - out[Chan_A]); // UNDER

            // Only min the cutout Z if matte alpha is greater than the alpha threshold:
            if (out[Chan_A] >= EPSILONf)
            {
               if (segment.Zf > 0.0f)
                  out[Chan_CutoutZ] = std::min(segment.Zf, out[Chan_CutoutZ]);
            }

        }
        else
        {
            const float iBa = (1.0f - out[Chan_A]);
            if (segment.isAdditive() || segment.hasPartialSubpixelBinCoverage())
            {
                // Correct for alpha overshooting 1.0 and weigh additive contribution
                // down by the overshoot amount to avoid any brightening artifacts:
                if ((out[Chan_A] + color[Chan_A]) > 1.0f)
                {
                    const float correction = (iBa / color[Chan_A]);
                    foreach_channel(z, comp_channels_no_alpha)
                        out[z] += color[z]*correction;
                    out[Chan_A] = out[Chan_CutoutA] = 1.0f;
                }
                else
                {
                    foreach_channel(z, comp_channels)
                        out[z] += color[z];
                    out[Chan_CutoutA] += color[Chan_A];
                }
            }
            else
            {
                foreach_channel(z, comp_channels)
                    out[z] += color[z]*iBa;
                out[Chan_CutoutA] += color[Chan_A]*iBa;
            }

            // Only min the Zs if undered alpha result is greater than the alpha threshold:
            if (out[Chan_A] >= EPSILONf)
            {
                if (segment.Zf > 0.0f)
                    /*out[Chan_Z] = */out[Chan_ZFront] = std::min(segment.Zf, out[Chan_ZFront]);
                if (segment.Zb > 0.0f)
                    out[Chan_ZBack] = std::max(segment.Zb, out[Chan_ZBack]);
            }

        }

#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout << "         COLOR:";
            std::cout.precision(6);
            foreach_channel(z, comp_channels)
                std::cout << " " << std::fixed << color[z];
            std::cout << std::endl;
            std::cout << "           OUT:";
            std::cout.precision(6);
            foreach_channel(z, comp_channels_with_cutout)
                std::cout << " " << std::fixed << out[z];
            std::cout << std::endl;
        }
#endif

        if (out[Chan_A] >= (1.0f - EPSILONf))
            break; // alpha saturated, all done

    } // nSegments

    // If nearest cutout Z is in front of non-cutout, output INF:
    if (out[Chan_CutoutZ] < out[Chan_ZFront])
        /*out[Chan_Z] = */out[Chan_ZFront] = out[Chan_ZBack] = INFINITYf;

    else if (out[Chan_ZBack] < 0.0f)
        out[Chan_ZBack] = INFINITYf;

    // Final alpha is cutout-alpha channel:
    out[Chan_A] = (out[Chan_CutoutA] >= (1.0f - EPSILONf))?1.0f:out[Chan_CutoutA];

} // DeepPixel::flattenNoOverlaps


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


typedef std::set<uint32_t> SegmentEdgeSet;

//
// Two of these are created for each DeepSegment to track
// the active segments.
//

enum { THIN_EDGE = -1, FRONT_EDGE = 0, BACK_EDGE = 1 };

struct SegmentEdge
{
    float      depth;
    uint32_t   segment;
    int        type;

    SegmentEdge(float _depth, uint32_t _segment, int _type) :
        depth(_depth),
        segment(_segment),
        type(_type)
    {
        //
    }

    bool operator < (const SegmentEdge& b) const
    {
        if (depth   < b.depth  ) return true;
        if (depth   > b.depth  ) return false;
        if (segment < b.segment) return true;
        if (segment > b.segment) return false;
        return (type < b.type);
    }
};


//
//  Flatten a list of sorted segments with overlap handling.
//

void
DeepPixel::flattenOverlapping (const ChannelSet& out_channels, Pixelf& out,
                               const SpMask8& spmask,
                               InterpolationMode interpolation)
{
    out.erase(out_channels);
    // Always fill in these output channels even though they may not be enabled
    // in the output pixel's channel set:
    //out[Chan_Z      ] =  INFINITYf;
    out[Chan_ZFront ] =  INFINITYf;
    out[Chan_ZBack  ] = -INFINITYf;
    out[Chan_CutoutA] =  0.0f;
    out[Chan_CutoutZ] =  INFINITYf;

    const uint32_t nSegments = m_segments.size();
#ifdef DCX_DEBUG_FLATTENER
    if (debug) {
//        std::cout << "    DeepPixel::flattenOverlapping=" << out_channels << ", nSegments=" << nSegments;
        std::cout << ", sp_mask=" << std::hex << spmask << std::dec << std::endl;
    }
#endif
    if (nSegments == 0)
    {
        out[Chan_ZBack] = INFINITYf;
        return; // shouldn't happen...
    }

    // Get the channel set to composite:
    ChannelSet comp_channels(out_channels);
    comp_channels &= m_channels; // only bother compositing requested output channels
    comp_channels += Chan_A;     // but, always composite alpha even if not requested
    comp_channels -= Mask_Depth; // don't composite depth channels, we handle those separately
    //
    ChannelSet comp_channels_no_alpha(comp_channels);
    comp_channels_no_alpha -= Chan_A;
    //
    ChannelSet comp_channels_with_cutout(comp_channels);
    comp_channels_with_cutout += Chan_CutoutA;
    ChannelSet comp_channels_with_cutout_no_alpha(comp_channels_no_alpha);
    comp_channels_with_cutout_no_alpha += Chan_CutoutA;

    // Is this deep pixel full of legacy (no spmask, no interpolation flag) samples?
    const bool useSpMasks = !isLegacyDeepPixel();

    // Force log interpolation in legacy mode:
    if (!useSpMasks)
        interpolation = INTERP_LOG;
#ifdef DCX_DEBUG_FLATTENER
    if (debug)
        std::cout << "      useSpMasks=" << useSpMasks << ", interpolation=" << interpolation << std::endl;
#endif

    // Build the list of SegmentEdges from DeepSegments:
    std::vector<SegmentEdge> segment_edges;
    segment_edges.reserve(nSegments * 2);
    for (uint32_t j=0; j < nSegments; ++j)
    {
        const DeepSegment& segment = m_segments[j];
#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout.precision(8);
            std::cout << "     " << j << ": Zf=" << segment.Zf << " Zb=" << segment.Zb;
            std::cout << " flags=("; segment.printFlags(std::cout); std::cout << "), mask=" << std::hex << segment.spMask() << std::dec;
            std::cout << " segment_thickness=" << (segment.Zb - segment.Zf) << std::endl;
            if (useSpMasks)
                segment.spMask().printPattern(std::cout, "        ");
        }
#endif

        // Skip segment if not in spmask:
        if (useSpMasks && !segment.maskBitsEnabled(spmask))
            continue;

        if (!isinf(segment.Zf) || !isinf(segment.Zb))
        {
            if (segment.isThin())
                segment_edges.push_back(SegmentEdge(segment.Zf, j, THIN_EDGE ));

            else
            {
                segment_edges.push_back(SegmentEdge(segment.Zf, j, FRONT_EDGE));
                segment_edges.push_back(SegmentEdge(segment.Zb, j, BACK_EDGE ));
            }
        }
    }
    const uint32_t nEdges = segment_edges.size();
    if (nEdges == 0)
    {
        out[Chan_ZBack] = INFINITYf;
        return; // No valid segments to combine, shouldn't happen...
    }

    // Re-sort edges, this will change order based on edge type:
    std::sort(segment_edges.begin(), segment_edges.end());

    SegmentEdgeSet active_segments;
    int num_log_samples = 0;
    int num_lin_samples = 0;
    int num_additive_samples = 0;

    std::vector<Pixelf> sample_colors;
    sample_colors.reserve(10);
    std::vector<Pixelf> prev_colors;
    prev_colors.reserve(10);

    Pixelf    black_color(comp_channels_with_cutout); black_color.erase();
    Pixelf  section_color(comp_channels_with_cutout);
    Pixelf   interp_color(comp_channels_with_cutout);
    Pixelf   merged_color(comp_channels_with_cutout);
    Pixelf additive_color(comp_channels_with_cutout);

#ifdef DCX_DEBUG_FLATTENER
    if (debug) {
        std::cout << "      segment_edges:" << std::endl;
        std::cout.precision(8);
        for (uint32_t j=0; j < nEdges; ++j) {
            const SegmentEdge& edge = segment_edges[j];
            const DeepSegment& segment = m_segments[edge.segment];
            const Pixelf& pixel = getSegmentPixel(edge.segment);
            std::cout.precision(8);
            std::cout << "       " << j << ": segment=" << edge.segment << ", type=" << edge.type;
            std::cout << ", "; segment.printFlags(std::cout);
            std::cout << ", depth=" << edge.depth;
            std::cout.precision(6);
            foreach_channel(z, comp_channels)
                std::cout << " " << std::fixed << pixel[z];
            std::cout << std::endl;
        }
        std::cout << "      ----------------- edges loop -------------------" << std::endl;
    }
#endif
    for (uint32_t j=0; j < nEdges; ++j)
    {
        const SegmentEdge& edge = segment_edges[j];
#ifdef DCX_DEBUG_FLATTENER
        assert(edge.segment < nSegments);
#endif
        const DeepSegment& segment0 = m_segments[edge.segment];

#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout.precision(8);
            std::cout << "      e" << j << ": segment=" << edge.segment << ", type=" << edge.type;
            std::cout << ", "; segment0.printFlags(std::cout);
            std::cout << ", depth=" << edge.depth;
            std::cout.precision(6);
            const Pixelf& pixel0 = getSegmentPixel(edge.segment);
            foreach_channel(z, comp_channels)
                std::cout << " " << std::fixed << pixel0[z];
            std::cout << std::endl;
        }
#endif

        bool linear_interp0 = (interpolation == INTERP_LIN ||
                              (interpolation == INTERP_AUTO && segment0.isHardSurface()));

        // We need to keep track of which segment have started but not yet finished.
        // So, for each edge add and remove samples as appropriate:
        if (edge.type == FRONT_EDGE)
        {
            // Add samples on their front edge:
            active_segments.insert(edge.segment);
#ifdef DCX_DEBUG_FLATTENER
            if (debug) std::cout << "           ** insert front-edge **" << std::endl;
#endif
            if (segment0.isHardSurface()) ++num_lin_samples; else ++num_log_samples;
            if (segment0.isAdditive() || segment0.hasPartialSubpixelBinCoverage()) ++num_additive_samples;

        }
        else if (edge.type == BACK_EDGE)
        {
            // Remove samples on their back edge:
            active_segments.erase(edge.segment);
#ifdef DCX_DEBUG_FLATTENER
            if (debug) std::cout << "           ** remove back-edge **" << std::endl;
#endif
            if (segment0.isHardSurface()) --num_lin_samples; else --num_log_samples;
            if (segment0.isAdditive() || segment0.hasPartialSubpixelBinCoverage()) --num_additive_samples;

        }
        else if (edge.type == THIN_EDGE)
        {
            // For samples where front == back, don't bother merging:
#ifdef DCX_DEBUG_FLATTENER
            if (debug) std::cout << "           ** front == back, UNDER/PLUS immediate **" << std::endl;
#endif

            // UNDER or PLUS the value for each channel:
            const Pixelf& color = getSegmentPixel(edge.segment);
#ifdef DCX_DEBUG_FLATTENER
            if (debug) {
                std::cout.precision(6);
                std::cout << "            color[";
                foreach_channel(z, comp_channels_with_cutout)
                    std::cout << " " << color[z];
                std::cout << " ]" << std::endl;
            }
#endif

            if (segment0.isMatte())
            {
                // Matte object, color chans are black so just under alpha:
                if (segment0.isAdditive() || segment0.hasPartialSubpixelBinCoverage())
                    out[Chan_A] += color[Chan_A]; // ADDITIVE
                else
                    out[Chan_A] += color[Chan_A]*(1.0f - out[Chan_A]); // UNDER
                // Only min the cutout Z if matte alpha is greater than the alpha threshold:
                if (out[Chan_A] >= EPSILONf)
                {
                    if (segment0.Zf > 0.0f)
                        out[Chan_CutoutZ] = std::min(segment0.Zf, out[Chan_CutoutZ]);
                }

            }
            else
            {
                const float iBa = (1.0f - out[Chan_A]);
                if (segment0.isAdditive() || segment0.hasPartialSubpixelBinCoverage())
                {
                    // ADDITIVE MODE:
                    // Correct for alpha overshooting 1.0 and weigh additive contribution
                    // down by the overshoot amount to avoid any brightening artifacts:
                    if ((out[Chan_A] + color[Chan_A]) > 1.0f)
                    {
                        const float correction = (iBa / color[Chan_A]);
                        foreach_channel(z, comp_channels_no_alpha)
                            out[z] += color[z]*correction;
                        out[Chan_A] = 1.0f;
                        out[Chan_CutoutA] = 1.0f;
                    }
                    else
                    {
                        foreach_channel(z, comp_channels_no_alpha)
                            out[z] += color[z];
                        out[Chan_A] += color[Chan_A];
                        out[Chan_CutoutA] += color[Chan_A];
                    }
                }
                else
                {
                    foreach_channel(z, comp_channels_no_alpha)
                        out[z] += color[z]*iBa;
                    out[Chan_A] += color[Chan_A]*iBa;
                    out[Chan_CutoutA] += color[Chan_A]*iBa;
                }

                // Only min the Zs if undered alpha result is greater than the alpha threshold:
                if (out[Chan_A] >= EPSILONf)
                {
                    if (segment0.Zf > 0.0f)
                        /*out[Chan_Z] = */out[Chan_ZFront] = std::min(segment0.Zf, out[Chan_ZFront]);
                    if (segment0.Zb > 0.0f)
                        out[Chan_ZBack] = std::max(segment0.Zb, out[Chan_ZBack]);
                }

            }

#ifdef DCX_DEBUG_FLATTENER
            if (debug) {
                std::cout << "              OUT[";
                std::cout.precision(12);
                foreach_channel(z, comp_channels_with_cutout)
                    std::cout << " " << out[z];
                std::cout << " ]" << std::endl;
            }
#endif

            // If nearest cutout Z is in front of non-cutout, output INF:
            if (out[Chan_CutoutZ] < out[Chan_ZFront])
                /*out[Chan_Z] = */out[Chan_ZFront] = out[Chan_ZBack] = INFINITYf;

            else if (out[Chan_ZBack] < 0.0f)
                out[Chan_ZBack] = INFINITYf;

            // No need to add further segments if this one has solid alpha:
            if (color[Chan_A] >= (1.0f - EPSILONf))
            {
                out[Chan_A] = (out[Chan_CutoutA] >= (1.0f - EPSILONf))?1.0f:out[Chan_CutoutA];
                return;
            }
        }

        // No active edges?  Skip to next edge:
        if (active_segments.empty())
            continue;

        // There is always at least another entry in edges at this point, to disable the ones that are active.
        // In the final iteration in the iterator for-loop, active_segments should be empty!
#ifdef DCX_DEBUG_FLATTENER
        assert((j + 1) < nEdges); // shouldn't heppen...
#endif

        // Get Z distance between this edge and the next edge:
        const float Z0 = edge.depth;
        const float Z1 = segment_edges[j + 1].depth;
        const float distance_to_next_edge = (Z1 - Z0);

        // Segment too thin to interpolate - skip it:
        if (!linear_interp0 && distance_to_next_edge < EPSILONf)
            continue;

#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout << "        ----------------- merging loop -------------------" << std::endl;
            std::cout.precision(8);
            std::cout << "        combine active samples [";
            for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                std::cout << " ," << *it;
            std::cout << " ], Z0=" << Z0 << ", Z1=" << Z1 << ", distance_to_next_edge=" << distance_to_next_edge << std::endl;
        }
#endif

        bool all_matte = true;

        // If we have all log active samples and no linear active samples then we can
        // use Florian's log merge math, otherwise we need to step through the
        // sub-segment range evaluating the samples at each step distance and under them:

        if (interpolation == INTERP_LOG ||
            (interpolation == INTERP_AUTO && num_log_samples > 0 && num_lin_samples == 0))
        {
            //=========================================================================================
            //
            // All-log samples to merge - handle legacy vs. spmask deep pixel:
            //
            //=========================================================================================

            merged_color.erase(comp_channels_with_cutout);

            if (useSpMasks)
            {
                //==============================================================
                //
                // Merge ALL-LOG spmask samples - use Florian's deep merge math
                // converted for channel-loop use:
                //
                //==============================================================
                for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                {
                    const uint32_t active_segment = *it;
#ifdef DCX_DEBUG_FLATTENER
                    assert(active_segment < m_segments.size());
#endif

                    const DeepSegment& interp_segment = m_segments[active_segment];
                    const Pixelf& interp_pixel = getSegmentPixel(active_segment);

                    // Get sub-section chunk from segment:
                    const float segment_thickness = (interp_segment.Zb - interp_segment.Zf);
                    const float section_weight = distance_to_next_edge / segment_thickness;
#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                        std::cout.precision(8);
                        std::cout << "          NEW sa" << active_segment << ": segment[" << interp_segment.Zf << " " << interp_segment.Zb << "]";
                        std::cout << ", segment_thickness=" << segment_thickness << ", section_weight=" << section_weight << ", LOG INTERP" << std::endl;
                    }
#endif

                    // Get log-interpolated color:
                    if (interp_segment.isMatte())
                    {
                        // Matte object, blacken color channels:
                        interp_segment.interpolateLog(interp_pixel, Mask_A, section_weight, section_color);
                        section_color.erase(comp_channels_with_cutout_no_alpha);

                    }
                    else
                    {
                        all_matte = false;
                        interp_segment.interpolateLog(interp_pixel, comp_channels, section_weight, section_color);
                        section_color[Chan_CutoutA] = section_color[Chan_A];
                    }

                    // Volumetric log merging math from Florian's deep doc:
                    const float a0 =  merged_color[Chan_A];
                    const float a1 = section_color[Chan_A];
                    const float a_merged = (a0 + a1) - (a0 * a1);
                    if (a0 >= 1.0f && a1 >= 1.0f)
                    {
                        // Max opacity, average them:
                        foreach_channel(z, comp_channels_with_cutout)
                            merged_color[z] = (merged_color[z] + section_color[z]) / 2.0f;
                    }
                    else if (a0 >= 1.0f)
                    {
                        // No merge, leave as is
                    }
                    else if (a1 >= 1.0f)
                    {
                        // No merge, copy sample:
                        foreach_channel(z, comp_channels_with_cutout)
                            merged_color[z] = section_color[z];
                    }
                    else
                    {
                        // Log merge:
                        foreach_channel(z, comp_channels_with_cutout)
                        {
                            if (z == Chan_A)
                                continue;
                            static const float MAXF = std::numeric_limits<float>::max();
                            const float u1 = float(-log1p(-a0));
                            const float v1 = (u1 < a0*MAXF)?u1/a0:1.0f;
                            const float u2 = float(-log1p(-a1));
                            const float v2 = (u2 < a1*MAXF)?u2/a1:1.0f;
                            const float u = u1 + u2;
                            if (u > 1.0f || a_merged < u*MAXF)
                               merged_color[z] = (merged_color[z]*v1 + section_color[z]*v2)*(a_merged / u);
                            else
                               merged_color[z] = merged_color[z]*v1 + section_color[z]*v2;
                        }
                    }
                    merged_color[Chan_A] = a_merged;

#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                       std::cout.precision(6);
                       std::cout << "             section_color[";
                       foreach_channel(z, comp_channels_with_cutout)
                          std::cout << " " << std::fixed << section_color[z];
                       std::cout << " ]" << std::endl;
                       std::cout << "            merged_color[";
                       foreach_channel(z, comp_channels_with_cutout)
                          std::cout << " " << std::fixed << merged_color[z];
                       std::cout << " ]" << std::endl;
                    }
#endif

                } // active segments merge loop

            }
            else // useSpMasks?
            {
                //==================================================================================
                //
                // Merge ALL-LOG no-spmasks samples - calculate total absorption for merged
                // segments using Foundry-compatible (legacy Nuke) logic:
                //
                //==================================================================================

                double absorption_accum = 0.0;
                float  merged_alpha = 0.0f;
                for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                {
                    const uint32_t active_segment = *it;
#ifdef DCX_DEBUG_FLATTENER
                    assert(active_segment < m_segments.size());
#endif

                    const DeepSegment& interp_segment = m_segments[active_segment];
                    const Pixelf& interp_pixel = getSegmentPixel(active_segment);

                    // Get sub-section within segment:
                    const float segment_thickness  = (interp_segment.Zb - interp_segment.Zf);
                    const float section_weight = (segment_thickness < EPSILONf)?1.0f:(distance_to_next_edge / segment_thickness);

#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                        std::cout.precision(8);
                        std::cout << "          LEGACY sa" << active_segment << ": segment[" << interp_segment.Zf << " " << interp_segment.Zb << "]";
                        std::cout << ", segment_thickness=" << segment_thickness << ", section_weight=" << section_weight << ", LOG INTERP";
                    }
#endif

                    // Only apply alpha correction if alpha is a grey value:
                    const float interp_alpha = interp_pixel[Chan_A];
                    section_color = interp_pixel;
                    if (interp_segment.isMatte())
                    {
                        // Matte object, blacken color channels:
                        section_color.erase(comp_channels_with_cutout_no_alpha);
#ifdef DCX_DEBUG_FLATTENER
                    if (debug) std::cout << ", MATTE";
#endif

                    }
                    else
                    {
                        all_matte = false;
                        section_color[Chan_CutoutA] = section_color[Chan_A];
                    }

                    // Composite with final alpha for all samples at this segment:
                    if (interp_alpha <= 0.0f || interp_alpha >= 1.0f)
                    {
                        merged_alpha = merged_alpha*(1.0f - interp_alpha) + interp_alpha;
#ifdef DCX_DEBUG_FLATTENER
                        if (debug) std::cout << ", interp_alpha=" << interp_alpha << std::endl;
#endif
                    }
                    else
                    {
                        const float viz = 1.0f - CLAMP(interp_alpha);
                        const float section_alpha = 1.0f - powf(viz, section_weight);
                        const float correction = section_alpha / interp_alpha;
                        foreach_channel(z, comp_channels_with_cutout)
                            section_color[z] *= correction;
                        section_color[Chan_A] = section_alpha;

                        merged_alpha = merged_alpha*(1.0f - section_alpha) + section_alpha;
#ifdef DCX_DEBUG_FLATTENER
                        if (debug) std::cout << ", interp_alpha=" << interp_alpha << ", section_alpha=" << section_alpha << std::endl;
#endif
                    }

                    const float section_alpha = section_color[Chan_A];
                    if (section_alpha <= 0.0f)
                    {
                        // Alpha transparent, skip it

                    }
                    else if (section_alpha < 1.0f)
                    {
                        // Partially-transparent, find the absorbance-weighted average of the
                        // unpremultiplied section color:
                        const double absorbance = -log(double(1.0f - section_alpha));
                        const float inv_section_alpha = 1.0f / section_alpha;
                        foreach_channel(z, comp_channels_with_cutout)
                            merged_color[z] += float(double(section_color[z]*inv_section_alpha) * absorbance);
                        absorption_accum += absorbance;
                    }
                    else
                    {
                        // Alpha saturated, max absorbance:
                        absorption_accum += MAX_ABSORBANCE;
                    }

#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                        std::cout.precision(6);
                        std::cout << "             section_color[";
                        foreach_channel(z, comp_channels_with_cutout)
                            std::cout << " " << std::fixed << section_color[z];
                        std::cout << " ]" << std::endl;
                        std::cout << "            merged_color[";
                        foreach_channel(z, comp_channels_with_cutout)
                            std::cout << " " << std::fixed << merged_color[z];
                        std::cout << " ]" << std::endl;
                    }
#endif
                
                } // active segments merge loop

                // Weight final merged result by the accumulated absorption factor:
                if (absorption_accum < EPSILONd)
                {
                    foreach_channel(z, comp_channels_with_cutout)
                        merged_color[z] = 0.0f;
                }
                else
                {
                    absorption_accum = 1.0 / absorption_accum;
                    foreach_channel(z, comp_channels_with_cutout)
                        merged_color[z] = float(double(merged_color[z]) * absorption_accum)*merged_alpha;
                }
                merged_color[Chan_A] = merged_alpha;


#ifdef DCX_DEBUG_FLATTENER
                if (debug) {
                    std::cout.precision(6);
                    std::cout << "            merged_color[";
                    foreach_channel(z, comp_channels_with_cutout)
                        std::cout << " " << std::fixed << merged_color[z];
                    std::cout << " ]" << std::endl;
                }
#endif
            } // all-log-interpolation mode

            // UNDER or PLUS the value for each channel:
            if (segment0.isAdditive() || segment0.hasPartialSubpixelBinCoverage())
            {
                // ADDITIVE MODE:
                // Correct for alpha overshooting 1.0 and weigh additive contribution
                // down by the overshoot amount to avoid any brightening artifacts:
                if ((out[Chan_A] + merged_color[Chan_A]) > 1.0f)
                {
                    const float correction = (1.0f - out[Chan_A]) / merged_color[Chan_A];
                    foreach_channel(z, comp_channels_with_cutout)
                        out[z] += merged_color[z]*correction;
                    out[Chan_A] = 1.0f;
                }
                else
                {
                    foreach_channel(z, comp_channels_with_cutout)
                        out[z] += merged_color[z];
                }
            }
            else
            {
                const float iBa = (1.0f - out[Chan_A]);
                foreach_channel(z, comp_channels_with_cutout)
                    out[z] += merged_color[z]*iBa;
            }

#if 0
        }
        else if (num_log_samples == 0 && num_lin_samples > 0)
        {
            //-----------------------------------------------------------------------------------------
            // All-lin:
            //-----------------------------------------------------------------------------------------
            // (TODO: Do we need an implementation here...?)

            //merged_color.erase(comp_channels_with_cutout);

#endif

        }
        else
        {
            //-----------------------------------------------------------------------------------------
            // Log/Lin combo (or Lin/Lin, see All-lin note above):
            //-----------------------------------------------------------------------------------------

            // Only 1 sample and it's a lin?  Don't bother step sampling:
            if (num_log_samples == 0 && num_lin_samples == 1)
            {
                merged_color.erase(comp_channels_with_cutout);

                const uint32_t active_segment = *active_segments.begin();
#ifdef DCX_DEBUG_FLATTENER
                assert(active_segment < m_segments.size());
#endif

                const DeepSegment& interp_segment = m_segments[active_segment];
                const Pixelf& interp_pixel = getSegmentPixel(active_segment);
                //
                if (interp_segment.isMatte())
                {
                    // Matte object, blacken color channels:
                    section_color.erase(comp_channels_with_cutout_no_alpha);
                    section_color[Chan_A] = interp_pixel[Chan_A];

                }
                else
                {
                    all_matte = false;
                    foreach_channel(z, comp_channels)
                        section_color[z] = interp_pixel[z];
                    section_color[Chan_CutoutA] = section_color[Chan_A];
                }
                //
                const float segment_thickness = (interp_segment.Zb - interp_segment.Zf);
                //
                // Split segment twice to get the correct weight at Zb.
                // Interpolate at Z0 & Z1 and un-under Z1 from Z0:
                const float tf = (Z0 - interp_segment.Zf) / segment_thickness;
                const float tb = (Z1 - interp_segment.Zf) / segment_thickness;
                const float Ba = CLAMP(section_color[Chan_A])*tf;
                // Un-under:
                if (Ba <= 0.0f)
                    merged_color = (section_color*tb - section_color*tf);
                else if (Ba < 1.0f)
                    merged_color = (section_color*tb - section_color*tf) / (1.0f - Ba);
#ifdef DCX_DEBUG_FLATTENER
                if (debug) {
                    std::cout.precision(8);
                    std::cout << "          sa" << active_segment << ": segment[" << interp_segment.Zf << " " << interp_segment.Zb << "]";
                    std::cout << ", segment_thickness=" << segment_thickness << ", "; interp_segment.printFlags(std::cout);
                    std::cout << std::endl;
                }
#endif

                // UNDER or PLUS the value for each channel:
                if (interp_segment.isAdditive() || interp_segment.hasPartialSubpixelBinCoverage())
                {
                    // ADDITIVE MODE:
                    // Correct for alpha overshooting 1.0 and weigh additive contribution
                    // down by the overshoot amount to avoid any brightening artifacts:
                    if ((out[Chan_A] + merged_color[Chan_A]) > 1.0f)
                    {
                        const float correction = (1.0f - out[Chan_A]) / merged_color[Chan_A];
                        foreach_channel(z, comp_channels_with_cutout)
                            out[z] += merged_color[z]*correction;
                        out[Chan_A] = 1.0f;
                    }
                    else
                    {
                        foreach_channel(z, comp_channels_with_cutout)
                            out[z] += merged_color[z];
                    }

                }
                else
                {
                    const float iBa = (1.0f - out[Chan_A]);
                    foreach_channel(z, comp_channels_with_cutout)
                        out[z] += merged_color[z]*iBa;
                }

            }
            else
            {

                //----------------------------------------------------------------
                // Step through sub-segment sampling and UNDER-ing to find the
                // final merged value.
                //----------------------------------------------------------------

                /* TODO: make steps non-linear and/or adaptive - use a pow curve?

                   Mark's comment re. handling step size & count:
                     Well, since smaller steps reduce the error I would try to evaluate
                     (or rather estimate) what amount of error would be produced by
                     a given step size.  Think about the threshold we have used before
                     for deep image compression - 0.003 meaning that we don't want
                     individual samples to deviate more than this amount.  Unfortunately
                     this can result in a large number of steps.

                     It seems rational that samples with larger alpha contribution
                     would need to be broken up more.  I would also think that as
                     alpha increases in the flattened output we could get away with
                     a larger step size because the contribution is lower.  Furthermore
                     as flattened rgb increases you can increase step size because
                     perceptually the increments in rgb are less meaningful.)
                */
                // For now we're using a fixed number of steps:
                const double step_size = (double(Z1) - double(Z0)) / double(SEGMENT_SAMPLE_STEPS - 1);

                sample_colors.clear();
                sample_colors.reserve(active_segments.size());
                prev_colors.clear();
                prev_colors.reserve(active_segments.size());

                // Initialize the sample colors to the start of the sub-segment:
                for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                {
                    const uint32_t active_segment = *it;
#ifdef DCX_DEBUG_FLATTENER
                    assert(active_segment < nSegments);
#endif

                    const DeepSegment& interp_segment = m_segments[active_segment];
                    const Pixelf& interp_pixel = getSegmentPixel(active_segment);

                    const float segment_thickness = (interp_segment.Zb - interp_segment.Zf);
#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                       std::cout.precision(8);
                       std::cout << "          sa" << active_segment << ": segment[" << interp_segment.Zf << " " << interp_segment.Zb << "]";
                       std::cout << ", segment_thickness=" << segment_thickness << std::endl;
                    }
#endif

                    if (interp_segment.isHardSurface())
                    {
                        // Lin - get the raw sample colors:
                        if (interp_segment.isMatte())
                        {
                           // Matte object, blacken color channels:
                           section_color.erase(comp_channels_with_cutout_no_alpha);
                           section_color[Chan_A] = interp_pixel[Chan_A];

                        }
                        else
                        {
                           all_matte = false;
                           foreach_channel(z, comp_channels)
                              section_color[z] = interp_pixel[z];
                           section_color[Chan_CutoutA] = section_color[Chan_A];
                        }
#ifdef DCX_DEBUG_FLATTENER
                        if (debug) {
                           std::cout.precision(6);
                           std::cout << "            i0: s" << active_segment << " LIN INITIAL[";
                           foreach_channel(z, comp_channels_with_cutout)
                              std::cout << " " << section_color[z];
                           std::cout << " ]" << std::endl;
                        }
#endif
                        sample_colors.push_back(section_color);
                        // And the first interpolated color at Zf:
                        const float t = (Z0 - interp_segment.Zf) / segment_thickness;
                        prev_colors.push_back(section_color * t);

                    }
                    else
                    {
                        // Log - each step has the same thickness so calc it once here:
                        const float section_weight = float(step_size / segment_thickness);
                        if (interp_segment.isMatte())
                        {
                           // Matte object, interpolate alpha and blacken color channels:
                           interp_segment.interpolateLog(interp_pixel, Mask_A, section_weight, section_color);
                           section_color.erase(comp_channels_with_cutout_no_alpha);

                        }
                        else
                        {
                           all_matte = false;
                           interp_segment.interpolateLog(interp_pixel, comp_channels, section_weight, section_color);
                           section_color[Chan_CutoutA] = section_color[Chan_A];
                        }
#ifdef DCX_DEBUG_FLATTENER
                        if (debug) {
                           std::cout.precision(6);
                           std::cout << "            i0: s" << active_segment << " LOG INITIAL[";
                           foreach_channel(z, comp_channels_with_cutout)
                              std::cout << " " << section_color[z];
                           std::cout << " ]" << std::endl;
                        }
#endif
                        sample_colors.push_back(section_color);
                        prev_colors.push_back(black_color);
                    }
                } // active segments loop
#ifdef DCX_DEBUG_FLATTENER
                if (debug) std::cout << "            --------" << std::endl;
#endif

                /*
                    Handling additive segments:
                    1) Build two lists of active segments, one for additive and non-additive segments
                    2) At each slice step sample the segments, but for additive segments accumulate by adding them
                       together instead of merging them
                    3) As each segment is built up by each step check if additive samples have saturated (A >= 1) and
                       knock down other channels by the amount alpha has exceeded 1.0.
                    4) Add the additive result to the non-additive result for final output

                */


                // Step through the Z0-Z1 range interpolating the active segments, averaging them together to
                // make one slice, then UNDER-ing the slice:
                for (uint32_t i=1; i < SEGMENT_SAMPLE_STEPS; ++i)
                {
                    uint32_t interp_index = 0;
                    merged_color.erase(comp_channels_with_cutout);
                    additive_color.erase(comp_channels_with_cutout);
                    float accum_alpha = 0.0f;
                    //float merged_alpha = 0.0f;
                    int additive_count = 0;
                    for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it, ++interp_index)
                    {
                        const uint32_t active_segment = *it;
#ifdef DCX_DEBUG_FLATTENER
                        assert(active_segment < nSegments);
#endif

                        const DeepSegment& interp_segment = m_segments[active_segment];
                        //const Pixelf& interp_pixel = getSegmentPixel(active_segment);

                        const double segment_thickness = (double(interp_segment.Zb) - double(interp_segment.Zf));
                        //
                        // Handle the interpolation mode:
                        if (interp_segment.isHardSurface())
                        {
                            //-----------------------------------------------------------------------------------------
                            // Linear interpolation
                            //-----------------------------------------------------------------------------------------
                            // Interpolate to Zt, and un-under Zt from previous:
                            const double Zt = double(Z0) + double(i)*step_size;
                            const float t = float((Zt - double(interp_segment.Zf)) / segment_thickness);
                            interp_color = sample_colors[interp_index]*t;
                            Pixelf& prev_color  = prev_colors[interp_index];
                            const float Ba = prev_color[Chan_A];
                            // UN-UNDER:
                            if (Ba <= 0.0f)
                                section_color = (interp_color - prev_color);
                            else if (Ba < 1.0f)
                                section_color = (interp_color - prev_color) / (1.0f - Ba);
                            else
                                section_color = black_color;
#ifdef DCX_DEBUG_FLATTENER
                            if (debug) {
                                std::cout << "            i" << i << ": s" << active_segment << " "; interp_segment.printFlags(std::cout); std::cout << ":";
                                std::cout.precision(6);
                                std::cout << "  interp_color[";
                                foreach_channel(z, comp_channels_with_cutout)
                                    std::cout << " " << interp_color[z];
                                std::cout << " ]";
                            }
#endif
                            prev_color = interp_color;

                        }
                        else
                        {
                            //-----------------------------------------------------------------------------------------
                            // Log interpolation - use back section of segment
                            //-----------------------------------------------------------------------------------------
#ifdef DCX_DEBUG_FLATTENER
                            if (debug) {
                                std::cout << "            i" << i << ": s" << active_segment << " "; interp_segment.printFlags(std::cout); std::cout << ":";
                            }
#endif
                            section_color = sample_colors[interp_index];
                        }
#ifdef DCX_DEBUG_FLATTENER
                        if (debug) {
                            std::cout.precision(6);
                            std::cout << " section_color[";
                            foreach_channel(z, comp_channels_with_cutout)
                                std::cout << " " << section_color[z];
                            std::cout << " ]" << std::endl;
                        }
#endif

                        if (interp_segment.isAdditive() || interp_segment.hasPartialSubpixelBinCoverage())
                        {
                            additive_color += section_color;
                            ++additive_count;

                        }
                        else
                        {
                            merged_color += section_color * section_color[Chan_A];
                            accum_alpha += section_color[Chan_A];
                            //merged_alpha = merged_alpha*(1.0f - section_color[Chan_A]) + section_color[Chan_A];
                        }

                    } // active segments loop

                    if (accum_alpha > EPSILONf)
                        merged_color /= accum_alpha;
#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                        std::cout.precision(6);
                        std::cout << "                merged_color[";
                        foreach_channel(z, comp_channels_with_cutout)
                            std::cout << " " << merged_color[z];
                        std::cout << " ]" << std::endl;
                        if (additive_count > 0) {
                            std::cout << "              additive_color[";
                            foreach_channel(z, comp_channels_with_cutout)
                                std::cout << " " << additive_color[z];
                            std::cout << " ]" << std::endl;
                        }
                    }
#endif

                    const float iBa = (1.0f - out[Chan_A]);
                    if (additive_count > 0)
                    {
                        if ((out[Chan_A] + additive_color[Chan_A]) > 1.0f)
                        {
                            // Correct for alpha overshooting 1.0 by attenuating additive contribution
                            // by the overshoot amount to avoid any brightening artifacts:
                            const float correction = (iBa / additive_color[Chan_A]);
                            foreach_channel(z, comp_channels_with_cutout)
                                out[z] += (merged_color[z]*iBa) + additive_color[z]*correction;
                            out[Chan_A] = 1.0f;
                        }
                        else
                        {
                            foreach_channel(z, comp_channels_with_cutout)
                                out[z] += (merged_color[z]*iBa) + additive_color[z];
                        }

                    }
                    else
                    {
                        foreach_channel(z, comp_channels_with_cutout)
                            out[z] += merged_color[z]*iBa;
                    }

#ifdef DCX_DEBUG_FLATTENER
                    if (debug) {
                        std::cout << "                         OUT[";
                        std::cout.precision(6);
                        foreach_channel(z, comp_channels_with_cutout)
                            std::cout << " " << out[z];
                        std::cout << " ]" << std::endl;
                    }
#endif
        
                    // If final alpha is now saturated we're done:
                    if (out[Chan_A] >= (1.0f - EPSILONf))
                    {
                        out[Chan_A] = 1.0f;
                        break;
                    }

                } // SEGMENT_SAMPLE_STEPS loop

            } // log/lin c&& active-samples > 1

        } // lin/log mode

        if (out[Chan_A] >= EPSILONf)
        {
            // Only min the cutout Z if matte alpha is greater than the alpha threshold:
            if (all_matte)
            {
                if (Z0 > 0.0f)
                    out[Chan_CutoutZ] = std::min(Z0, out[Chan_CutoutZ]);
            }
            else
            {
                if (Z0 > 0.0f)
                    /*out[Chan_Z] = */out[Chan_ZFront] = std::min(Z0, out[Chan_ZFront]);
                if (Z1 > 0.0f)
                    out[Chan_ZBack] = std::max(Z1, out[Chan_ZBack]);
            }
        }

#ifdef DCX_DEBUG_FLATTENER
        if (debug) {
            std::cout << "              OUT[";
            std::cout.precision(12);
            foreach_channel(z, comp_channels_with_cutout)
                std::cout << " " << out[z];
            std::cout << " ]" << std::endl;
        }
#endif

        // If alpha is now saturated we're done:
        if (out[Chan_A] >= (1.0f - EPSILONf))
        {
#ifdef DCX_DEBUG_FLATTENER
            if (debug) std::cout << "          saturated, stop" << std::endl;
            // Clear the active_segments list so the exit asserts don't fail:
            active_segments.clear();
            num_log_samples = num_lin_samples = num_additive_samples = 0;
#endif
            break;
        }

    } // for nSegments loop

    // If nearest cutout Z is in front of non-cutout, output INF:
    if (out[Chan_CutoutZ] < out[Chan_ZFront])
        /*out[Chan_Z] = */out[Chan_ZFront] = out[Chan_ZBack] = INFINITYf;

    else if (out[Chan_ZBack] < 0.0f)
        out[Chan_ZBack] = INFINITYf;

    // Final alpha is cutout-alpha channel:
    out[Chan_A] = (out[Chan_CutoutA] >= (1.0f - EPSILONf))?1.0f:out[Chan_CutoutA];

#ifdef DCX_DEBUG_FLATTENER
    assert(active_segments.empty());
    assert(num_log_samples == 0);
    assert(num_lin_samples == 0);
    assert(num_additive_samples == 0);
#endif

} // DeepPixel::flattenOverlapping


//----------------------------------------------------------------------------

OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
