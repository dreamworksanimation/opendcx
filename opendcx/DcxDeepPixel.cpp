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

// Handle the Z accumulation with coverage weighting - i.e. pick Z from
// the sample with largest coverage.
// Finishing this code will likely produce more accurate flattened Z's:
#define DCX_USE_LARGEST_COVERAGE_Z 1


// Uncomment this to get debug info about DeepPixel:
//#define DCX_DEBUG_DEEPPIXEL 1
//#define DCX_DEBUG_COLLAPSING 1
//#define DCX_DEBUG_FLATTENER 1

#if defined(DCX_DEBUG_DEEPPIXEL) || defined(DCX_DEBUG_COLLAPSING) || defined(DCX_DEBUG_FLATTENER)
#  include <assert.h>
#  define SAMPLER_X -100000 // special debug coord
#  define SAMPLER_Y -100000 // special debug coord
#  define SAMPLER_Z 0
#  define SAMPLINGXY(A, B) (A==SAMPLER_X && B==SAMPLER_Y)
#  define SAMPLINGXYZ(A, B, C) (A==SAMPLER_X && B==SAMPLER_Y && C==SAMPLER_Z)
#endif

#include <map>

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


// For now we're using a fixed number of steps:
#ifdef DCX_DEBUG_FLATTENER
#  define SEGMENT_SAMPLE_STEPS 5
#else
#  define SEGMENT_SAMPLE_STEPS 5//15
#endif

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

/*friend*/
std::ostream&
operator << (std::ostream& os,
             const DeepFlags& flags)
{
    os << "0x";
    const std::ios_base::fmtflags osflags = os.flags();
    const char fill = os.fill();
    const int w = os.width();
    os.fill('0');
    os.width(16);
    os << std::hex << flags.bits;
    os.flags(osflags);
    os.fill(fill);
    os.width(w);

    return os;
}


void
DeepFlags::print (std::ostream& os) const
{
    if (bits == DeepFlags::ALL_BITS_OFF)
    {
        os << "none(Log,NoMatte)";
        return;
    }
    if (bits & Dcx::DeepFlags::LINEAR_INTERP)
        os << "Linear";
    else
        os << "Log";
    if (bits & Dcx::DeepFlags::MATTE_OBJECT)
        os << ",Matte";
    if (bits & Dcx::DeepFlags::ADDITIVE)
        os << ",Additive";
    if (hasPartialSpCoverage())
    {
        os.precision(3);
        os << ",spCvg(" << std::fixed << getSpCoverageWeight() << ")";
    }
}

//----------------------------------------------------------------------------------------

//
// Prints segment info and metadata, but no channel data.
//

void
DeepSegment::printInfo (std::ostream& os,
                        bool show_mask) const
{
    const std::streamsize prec = os.precision();
    os.precision(8);
    os << "Zf=" << Zf << ", Zb=" << Zb << ", flags=["; metadata.flags.print(os); os << "]";
    if (show_mask) {
        os << std::endl;
        metadata.spmask.printPattern(os, "  ");
    } else
        os << ", spMask=" << metadata.spmask;
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
    os << "{ xy[" << m_x << " " << m_y << "]";
    if (m_segments.size() > 0)
    {
        const std::ios_base::fmtflags flags = os.flags();
        const std::streamsize prec = os.precision();
        this->sort(true/*force*/); // Get global info up to date
        os << " overlaps=" << m_overlaps << " allFullCoverage=" << allFullCoverage();
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
                os << " spMask=" << segment.spMask();
            //
            os << " chans=" << getSegmentPixel(i) << std::endl;
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

/*friend*/
std::ostream&
operator << (std::ostream& os,
             const DeepMetadata& metadata)
{
    os << "[flags=" << metadata.flags;
    os << ", spCvg=" << metadata.getSpCoverageWeight();
    os << ", spMask=" << std::hex << metadata.spmask << std::dec << "]";
    return os;
}


//
// Outputs a raw list of values suitable for parsing.
//

/*friend*/
std::ostream&
operator << (std::ostream& os,
             const DeepSegment& ds)
{
    os << "[Zf=" << ds.Zf << ", Zb=" << ds.Zb << ", flags=" << ds.metadata.flags;
    os << ", spCvg=" << ds.metadata.getSpCoverageWeight();
    os << ", spMask=" << std::hex << ds.metadata.spmask << std::dec << ", coverage=" << ds.getCoverage() << "]";
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
                os << " " << dp.getChannel(i, z);
            os << " ]";
        }
    }
    os << "}";
    return os;
}


/*static*/
const char*
DeepPixel::interpolationModeString (InterpolationMode mode)
{
    switch (mode)
    {
    case INTERP_OFF:  return "off";     // Disable interpolation
    case INTERP_AUTO: return "auto";    // Determine interpolation from per-sample metadata (DeepFlags)
    case INTERP_LOG:  return "log";     // Use log interpolation for all samples
    case INTERP_LIN:  return "linear";  // Use linear interpolation for all samples
    default: return "invalid";
    }
}


//
//  Empty the segment list and clear shared values, except for channel set.
//

void
DeepPixel::clear ()
{
    m_segments.clear();
    m_pixels.clear();

    invalidateSort(); // force sort to re-run
}


//
//  Add a DeepSegment to the end of the list.
//

size_t
DeepPixel::append (const DeepSegment& bs,
                   const Pixelf& bp)
{
    const size_t new_segment_index = m_segments.size();
    const size_t new_pixel_index   = m_pixels.size();

    if (m_segments.capacity() < m_segments.size()+1)
        m_segments.reserve(m_segments.size() + (size_t)int(float(m_segments.size())/1.5f));
    if (m_pixels.capacity() < m_pixels.size()+1)
        m_pixels.reserve(m_pixels.size() + (size_t)int(float(m_pixels.size())/1.5f));
    m_segments.push_back(bs);
    m_pixels.push_back(bp);

    m_pixels[new_pixel_index].channels  = m_channels;
    m_segments[new_segment_index].index = (int)new_pixel_index;

    invalidateSort(); // force sort to re-run

    return new_segment_index;
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
#ifdef DCX_DEBUG_DEEPPIXEL
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
        if (!m_channels.contains(z))
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

    invalidateSort(); // force sort to re-run
}

//-------------------------------------------------------------------------------------

//
//  Remove a DeepSegment from the segment list, deleting its referenced Pixel.
//  Note that this method will possibly reorder some of the Pixel indices in the
//  DeepSegments, so a previously referenced Pixel index may become invalid and
//  need to be reaquired from its DeepSegment.
//

void DeepPixel::removeSegment (size_t segment_index)
{
    if (segment_index >= m_segments.size())
        return;
    const int overwritePixelIndex = m_segments[segment_index].index;
    const int lastPixelIndex = (m_pixels.size()-1);
    if (overwritePixelIndex < lastPixelIndex)
    {
        // Find the DeepSegment pointing at the last Pixel, then
        // copy the last Pixel in the list over the one being deleted
        // and update the index in the changed DeepSegment:
        const size_t nSegments = m_segments.size();
        size_t lastPixelSegmentIndex = 0;
        for (; lastPixelSegmentIndex < nSegments; ++lastPixelSegmentIndex)
            if (m_segments[lastPixelSegmentIndex].index == lastPixelIndex)
                break;
#ifdef DCX_DEBUG_DEEPPIXEL
        assert(lastPixelSegmentIndex < nSegments); // shouldn't happen...
#endif
        m_pixels[overwritePixelIndex] = m_pixels[lastPixelIndex];
        m_segments[lastPixelSegmentIndex].index = overwritePixelIndex;
    }
    // Delete the DeepSegment and the last Pixel:
    m_segments.erase(m_segments.begin()+segment_index);
    m_pixels.pop_back();
#ifdef DCX_DEBUG_DEEPPIXEL
    assert(m_segments.size() == m_pixels.size());
#endif
    invalidateSort(); // force sort to re-run
}


//-------------------------------------------------------------------------------------

//
//  Return the index of the DeepSegment nearest to Z and inside the distance
//  of Z + or - maxDistance. Return -1 if nothing found.
//

// TODO: finish implementing this!
#if 0
int
DeepPixel::nearestSegment (double Z,
                           double maxDistance)
{
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
}
#endif


//
//  Sort the segments.  If the sorted flag is true this returns quickly.
//  This also updates global overlap and coverage flags.
//

void
DeepPixel::sort (bool force)
{
    if (m_sorted && !force)
        return;
    m_sorted = true;
    m_overlaps = false;
    const size_t nSegments = m_segments.size();
    if (nSegments == 0)
    {
        m_accum_or_mask  = m_accum_and_mask  = SpMask8::zeroCoverage;
        m_accum_or_flags = m_accum_and_flags = DeepFlags::ALL_BITS_OFF;
        return;
    }

    // Z-sort the segments:
    std::sort(m_segments.begin(), m_segments.end());

    const DeepSegment& segment0 = m_segments[0];

    float prev_Zf = segment0.Zf;
    float prev_Zb = segment0.Zb;
    // Make sure mixed legacy full-coverage(allBitsOff) and true full-coverage(allBitsOn)
    // segments are both considered full-coverage:
    m_accum_or_mask  = m_accum_and_mask  = (segment0.fullCoverage())?SpMask8::fullCoverage:segment0.spMask();
    m_accum_or_flags = m_accum_and_flags = segment0.flags();

    // Determine global overlap and coverage status.
    for (size_t i=1; i < nSegments; ++i)
    {
        const DeepSegment& segment = m_segments[i];
        if (segment.Zf < prev_Zf || segment.Zb < prev_Zf ||
            segment.Zf < prev_Zb || segment.Zb < prev_Zb)
            m_overlaps = true;

        const SpMask8 spmask = (segment.fullCoverage())?SpMask8::fullCoverage:segment.spMask();
        m_accum_or_mask   |= spmask;
        m_accum_and_mask  &= spmask;
        m_accum_or_flags  |= segment.flags();
        m_accum_and_flags &= segment.flags();

        prev_Zf = segment.Zf;
        prev_Zb = segment.Zb;
    }
}

//
// Force sort to re-run.
//

void DeepPixel::invalidateSort ()
{
    m_sorted = m_overlaps = false;
    m_accum_or_mask  = m_accum_and_mask  = SpMask8::zeroCoverage;
    m_accum_or_flags = m_accum_and_flags = DeepFlags::ALL_BITS_OFF;
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
    return (m_accum_and_mask == SpMask8::fullCoverage || m_accum_or_mask == SpMask8::zeroCoverage);
}

bool
DeepPixel::allVolumetric ()
{
    sort(); // updates flags
    return ((m_accum_and_flags & DeepFlags::LINEAR_INTERP)==0 &&
            (m_accum_or_flags  & DeepFlags::LINEAR_INTERP)==0);
}
bool
DeepPixel::anyVolumetric ()
{
    sort(); // updates flags
    return (m_accum_or_flags & DeepFlags::LINEAR_INTERP)==0;
}

bool
DeepPixel::allHardSurface ()
{
    sort(); // updates flags
    return (m_accum_and_flags & DeepFlags::LINEAR_INTERP)!=0;
}
bool
DeepPixel::anyHardSurface ()
{
    sort(); // updates flags
    return (m_accum_or_flags & DeepFlags::LINEAR_INTERP)!=0;
}

bool
DeepPixel::allMatte ()
{
    sort(); // updates flags
    return (m_accum_and_flags & DeepFlags::MATTE_OBJECT)!=0;
}
bool
DeepPixel::anyMatte ()
{
    sort(); // updates flags
    return (m_accum_or_flags & DeepFlags::MATTE_OBJECT)!=0;
}

bool
DeepPixel::isLegacyDeepPixel ()
{
    sort(); // updates flags
    return (allZeroCoverage() && allVolumetric());
}


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

int
DeepPixel::findNearestMatch (float Zf, float Zb,
                             const SpMask8& match_spmask,
                             DeepFlags match_flags,
                             uint32_t start_index,
                             float depth_threshold)
{
    const size_t nSegments = m_segments.size();
    if (nSegments == 0 || start_index >= nSegments)
        return -1;

    sort();
    for (; start_index < nSegments; ++start_index)
    {
        const DeepSegment& segment = m_segments[start_index];
        if (match_flags!=0 && (segment.flags() & match_flags)==0 ||
            match_flags==0 && segment.flags()!=0)
            continue;
        else if (match_spmask!=SpMask8::fullCoverage && (match_spmask & segment.spMask())==0)
            continue;
        else if (fabsf(segment.Zf - Zf) <= depth_threshold &&
                 fabsf(segment.Zb - Zb) <= depth_threshold)
            return start_index;
    }

    return -1;
}

int
DeepPixel::findBestMatch (const DeepSegment& bs,
                          const Pixelf& bp,
                          const ChannelSet& compare_channels,
                          const SpMask8& match_spmask,
                          DeepFlags match_flags,
                          uint32_t start_index,
                          float color_threshold,
                          float depth_threshold)
{
    ChannelSet compare = compare_channels;
    compare &= bp.channels;
    compare &= m_channels;
    while (1) {
        const int zmatch = findNearestMatch(bs.Zf, bs.Zb, match_spmask, match_flags, start_index, depth_threshold);
        if (zmatch < 0)
            return -1; // no z-match
        // Compare color channels:
        const Pixelf& ap = m_pixels[m_segments[zmatch].index];
        bool cmatched = true;
        foreach_channel(z, compare) {
            if (fabsf(ap[z] - bp[z]) > color_threshold) {
                cmatched = false;
                break;
            }
        }
        if (cmatched)
            return zmatch;
        start_index = zmatch+1;
    }
}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

void
DeepPixel::findNearestMatches (float Zf, float Zb,
                               float depth_threshold,
                               std::vector<int>& matched_segments_list)
{
    matched_segments_list.clear();
    sort();
    const size_t nSegments = size();
    for (size_t i=0; i < nSegments; ++i)
    {
        const DeepSegment& segment = m_segments[i];
        // Bail fast if we go outside possible depth range:
        if ((segment.Zf - depth_threshold) > Zb)
            return;
        // If one of the Z's is outside threshold skip it:
        if (fabsf(segment.Zf - Zf) > depth_threshold || fabsf(segment.Zb - Zb) > depth_threshold)
            continue;
        matched_segments_list.push_back(i);
    }
}

void
DeepPixel::findNearestMatches (const DeepSegment& segment,
                               float depth_threshold,
                               std::vector<int>& matched_segments_list)
{
    findNearestMatches(segment.Zf, segment.Zb, depth_threshold, matched_segments_list);
}



inline
int
findSurfaceMatch (Dcx::DeepPixel& dp,
                  const std::vector<int>& segment_indices,
                  const SpMask8& match_spmask,
                  const DeepFlags& match_flags,
                  int   match_spbin_count)
{
    const size_t nZMatches = segment_indices.size();
    for (size_t i=0; i < nZMatches; ++i)
    {
        const int index = segment_indices[i];
        if (index < 0 || index >= (int)dp.size())
            continue;
        const DeepSegment& segment = dp.getSegment(index);
        // Compare flags and partial-counts:
        if (segment.flags() != match_flags ||
            (match_spbin_count >= 0 && (int)segment.getSpCoverageCount() != match_spbin_count))
            continue;
        else if (match_spmask!=SpMask8::fullCoverage && !(match_spmask & segment.spMask()))
            continue;
        return i;
    }

    return -1;
}

inline
int
findBestMatch2 (Dcx::DeepPixel& dp,
               const std::vector<int>& segment_indices,
               const Pixelf& color,
               const ChannelSet& compare_channels,
               const SpMask8& match_spmask,
               const DeepFlags& match_flags,
               int   match_spbin_count,
               float color_threshold)
{
    ChannelSet compare = compare_channels;
    compare &= dp.channels();
    const size_t nZMatches = segment_indices.size();
    for (size_t i=0; i < nZMatches; ++i)
    {
        const int index = segment_indices[i];
        if (index < 0 || index >= (int)dp.size())
            continue;
        const DeepSegment& segment = dp.getSegment(index);
        // Compare flags and partial-counts:
        if (segment.flags() != match_flags ||
            (match_spbin_count >= 0 && (int)segment.getSpCoverageCount() != match_spbin_count))
            continue;
        else if (match_spmask!=SpMask8::fullCoverage && !(match_spmask & segment.spMask()))
            continue;
        // Compare color channels:
        const Pixelf& ap = dp.getSegmentPixel(segment);
        bool cmatched = true;
        foreach_channel(z, compare) {
            if (fabsf(ap[z] - color[z]) > color_threshold) {
                cmatched = false;
                break;
            }
        }
        if (cmatched)
            return segment_indices[i];
    }

    return -1;
}


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------



//
// Add a partial subpixel-coverage segment to the DeepPixel, first searching
// for matches to combine with.
//
// When matches occur the matched segment can be split into two where the original
// segment has the new segment added to it with the matched bits, and the duplicated
// segment containing the non-matched bits from the original matched segment.
//

void
DeepPixel::appendOrCombineSegment (const DeepSegment& in_segment,
                                   const Pixelf& in_pixel,
                                   float depth_threshold,
                                   float color_threshold)
{
#ifdef DCX_DEBUG_COLLAPSING
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable)
        std::cout << "      appendOrCombineSegment()";
#endif

    // Find all the possible Z matches so that it's fast to repeatedly test
    // other segment parameters:
    std::vector<int> segment_indices;
    segment_indices.reserve(10);

    findNearestMatches(in_segment, depth_threshold, segment_indices);

    // If there's no Z-range matches don't bother trying to combine:
    if (segment_indices.size() == 0)
    {
#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable) {
            std::cout << std::endl;
            in_segment.spMask().printPattern(std::cout, "      ");
            std::cout << "        NO MATCH - add new segment" << std::endl;
        }
#endif
        append(in_segment, in_pixel);
    }
    else
    {
#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable) {
            std::cout << ": Zmatches[";
            for (size_t i=0; i < segment_indices.size(); ++i)
                std::cout << " " << segment_indices[i];
            std::cout << " ],";
        }
#endif
        appendOrCombineSegment(segment_indices, in_segment, in_pixel, color_threshold);
    }

}


//
// Add a partial subpixel-coverage segment to the DeepPixel, first searching
// for matches to combine within a subset list of existing segments.
//
// When matches occur the matched segment can be split into two where the original
// segment has the new segment added to it with the matched bits, and the duplicated
// segment containing the non-matched bits from the original matched segment.
//

void
DeepPixel::appendOrCombineSegment (const std::vector<int>& segment_indices,
                                   const DeepSegment& in_segment,
                                   const Pixelf& in_pixel,
                                   float color_threshold)
{
    const bool in_full_spcoverage = in_segment.hasFullSpCoverage();
#ifdef DCX_DEBUG_COLLAPSING
    //std::cout << "xy[" << m_x << " " << m_y << "]" << std::endl;
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable) {
        std::cout << " flags=[";
        in_segment.printFlags(std::cout);
        std::cout << "] chans=" << in_pixel << std::endl;
        in_segment.spMask().printPattern(std::cout, "      ");
    }
#endif

    // First try to match the color channels, flags and partial-weights.
    // If there's a match we can bail quickly.
    const int matched_values_index = findBestMatch2(*this,
                                                   segment_indices,
                                                   in_pixel,
                                                   in_pixel.channels,
                                                   SpMask8::fullCoverage,
                                                   in_segment.flags(),
                                                   in_segment.getSpCoverageCount(),
                                                   color_threshold);
    // If there's a values match determine if we ADD or OR the segments:
    if (matched_values_index >= 0)
    {
        //------------------------------------------
        // --------------   MATCH  -----------------
        //------------------------------------------
        DeepSegment& matched_segment = m_segments[matched_values_index];

        //--------------------------------------------------------------------------------
        // Full spcoverage?  OR the masks and we're done:
        //--------------------------------------------------------------------------------
        if (in_full_spcoverage)
        {
            matched_segment.metadata.spmask |= in_segment.spMask();
#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) {
                std::cout << "        MATCH - combined full segment:" << std::endl;
                matched_segment.spMask().printPattern(std::cout, "        ");
            }
#endif
            return;
        }

        //--------------------------------------------------------------------------------
        // Partial spcoverage?  The bits that overlap ADD together while the bits
        // that don't OR together.
        // If there's no overlapping bits we can OR them and bail.
        // Overlapping masks case is handled like a no-match:
        //--------------------------------------------------------------------------------

        const SpMask8 matched_bits = (matched_segment.spMask() &  in_segment.spMask());
        if (matched_bits == SpMask8::zeroCoverage)
        {
            // No overlap?  OR the masks and we're done:
            matched_segment.metadata.spmask |= in_segment.spMask();
#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) {
                std::cout << "        MATCH - combined partial segment:" << std::endl;
                matched_segment.spMask().printPattern(std::cout, "        ");
            }
#endif
            return;

        }

    }
    else
    {
        //------------------------------------------
        // -------------- NO MATCH -----------------
        //------------------------------------------

        //--------------------------------------------------------------------------------
        // Full spcoverage?  Add a new opaque segment and bail:
        //--------------------------------------------------------------------------------
        if (in_full_spcoverage)
        {
#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) std::cout << "        NO MATCH - new full segment" << std::endl;
#endif
            append(in_segment, in_pixel);
            return;
        }

    }

    //--------------------------------------------------------------------------------
    // No direct values match, but possibly combine PARTIAL spcoverage segments.
    //
    // There's two scenarios primary:
    // 1) Input partial segment's spmask completely or partly matches one of
    //      the existing partial segments. For the matching bits we add the
    //      segments together, and for the non-matching bits
    //
    // 2) Input partial segment's spmask don't coincide at all, match 
    //--------------------------------------------------------------------------------

    const size_t nAllZMatches = segment_indices.size();

    // Get the subset of Z matches that are partial and full and can safely combine with this segment:
    std::vector<int> partial_segment_indices;
    std::vector<int> full_segment_indices;
    partial_segment_indices.reserve(nAllZMatches);
    full_segment_indices.reserve(nAllZMatches);
    for (size_t i=0; i < nAllZMatches; ++i)
    {
        const int zmatched_index = segment_indices[i];
#ifdef DCX_DEBUG_COLLAPSING
        assert(zmatched_index >=0 && zmatched_index < (int)m_segments.size()); // shouldn't happen...
#else
        if (zmatched_index < 0 || zmatched_index >= (int)m_segments.size())
            continue;
#endif
        const DeepSegment& zmatched_segment = m_segments[zmatched_index];

        if (zmatched_segment.hasFullSpCoverage())
        {
            if ((zmatched_segment.flags() | DeepFlags::ADDITIVE) == in_segment.flags())
                full_segment_indices.push_back(segment_indices[i]);
            continue;
        }

        if (zmatched_segment.flags() != in_segment.flags() ||
            (zmatched_segment.getSpCoverageCount() + in_segment.getSpCoverageCount()) >= DeepFlags::maxSpCoverageCount)
            continue;
        partial_segment_indices.push_back(segment_indices[i]);
    }
    const size_t nPartialZMatches = partial_segment_indices.size();

    // No partial segments to combine with? Append new segment and bail:
    if (nPartialZMatches == 0)
    {
#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable) std::cout << "        NO MATCH - new partial segment" << std::endl;
#endif
        append(in_segment, in_pixel);
        return;
    }

    // Find partial-coverage segments that have spmask bits that overlap with the new
    // segments.
    // If there's no overlaps or there's remaining bits that didn't overlap then add
    // a new segment:
#ifdef DCX_DEBUG_COLLAPSING
    if (debug_enable) {
        std::cout << "        partialZmatches[";
        for (size_t i=0; i < nPartialZMatches; ++i)
            std::cout << " " << partial_segment_indices[i];
        std::cout << " ]";
        std::cout << " fullZmatches[";
        for (size_t i=0; i < full_segment_indices.size(); ++i)
            std::cout << " " << full_segment_indices[i];
        std::cout << " ]:" << std::endl;
    }
#endif

    DeepSegment new_segment;
    Pixelf new_pixel;

    // As matches are made this mask gets reduced:
    SpMask8 in_segment_bits = in_segment.spMask();

    for (size_t i=0; i < nPartialZMatches; ++i)
    {
        const int matched_segment_index = partial_segment_indices[i];
        DeepSegment& matched_segment = m_segments[matched_segment_index];

        const SpMask8 matched_bits = (matched_segment.spMask() & in_segment_bits);
        if (matched_bits == SpMask8::zeroCoverage)
            continue; // no overlaps

        Pixelf& matched_pixel = getSegmentPixel(matched_segment);

        const uint32_t bin_count_sum = matched_segment.getSpCoverageCount() + in_segment.getSpCoverageCount();

#ifdef DCX_DEBUG_COLLAPSING
        assert(bin_count_sum <= DeepFlags::maxSpCoverageCount);
        if (debug_enable) {
            std::cout << "        [ " << matched_segment_index << " ] Z-MATCH:";
            std::cout << " in_bin_count=" << in_segment.getSpCoverageCount();
            std::cout << ", matched_bin_count=" << matched_segment.getSpCoverageCount();
            std::cout << ", bin_count_sum=" << bin_count_sum;
            std::cout << std::endl;
            //std::cout << "          in_segment_bits:" << std::endl;
            //in_segment_bits.printPattern(std::cout, "          ");
            //std::cout << "          matched_bits:" << std::endl;
            //matched_bits.printPattern(std::cout, "          ");
            //std::cout << "          unmatched_bits:" << std::endl;
            //const SpMask8 unmatched_bits = (matched_segment.spMask() & ~in_segment_bits);
            //unmatched_bits.printPattern(std::cout, "          ");
        }
#endif

        // If all spmask bits match add the partial segments:
        if (matched_bits == matched_segment.spMask())
        {
            new_segment = matched_segment;
            new_pixel = matched_pixel;

#ifdef DCX_DEBUG_COLLAPSING
            if (debug_enable) std::cout << "        MATCHED BITS (new_bin_count=" << bin_count_sum << ")";
#endif
            // If adding the partials together causes alpha to saturate then we
            // correct for alpha overshooting 1.0 by attenuating additive contribution
            // by the overshoot amount to avoid any brightening artifacts.
            // Finally mark the segment as opaque vs. partial:
            if (bin_count_sum >= DeepFlags::maxSpCoverageCount)
            {
                // Add partial segments into opaque segment:
                new_segment.clearSpCoverageCount(); // turn off partial coverage
                if ((new_pixel[Chan_A] + in_pixel[Chan_A]) >= 1.0f)
                    new_pixel += in_pixel*((1.0f - new_pixel[Chan_A]) / in_pixel[Chan_A]);
                else
                    new_pixel += in_pixel;

                // We can possibly further combine this new opaque segment together
                // with the other opaque segments, but this time we would need to delete
                // one of the segments:
                const int matched_full_index = findBestMatch2(*this,
                                                             full_segment_indices,
                                                             new_pixel,
                                                             new_pixel.channels,
                                                             SpMask8::fullCoverage,
                                                             new_segment.flags(),
                                                             DeepFlags::maxSpCoverageCount,
                                                             color_threshold);
                // If match combine them into the new match by simply ORing the spmasks
                // then delete the currently matched segment, removing it from the list
                // of active matches:
                if (matched_full_index >= 0)
                {
                    DeepSegment& combined_segment = getSegment(matched_full_index);
                    combined_segment.metadata.spmask |= matched_bits;
                    removeSegment(matched_segment_index);
#ifdef DCX_DEBUG_COLLAPSING
                    if (debug_enable) {
                        std::cout << " - SECONDARY FULL MATCH: combine_segment=" << matched_full_index;
                        std::cout << ", remove_segment=" << matched_segment_index << std::endl;
                        combined_segment.spMask().printPattern(std::cout, "        ");
                    }
#endif
                }
                else
                {
                    // Save combined:
                    matched_segment = new_segment;
                    matched_pixel = new_pixel;
#ifdef DCX_DEBUG_COLLAPSING
                    if (debug_enable) std::cout << " -> opaque:" << std::endl;
#endif
                }

            }
            else
            {
                // Add partial segments:
                new_segment.setSpCoverageCount(bin_count_sum);
                new_pixel += in_pixel;
#ifdef DCX_DEBUG_COLLAPSING
                if (debug_enable) std::cout << " -> new weight=" << new_segment.getSpCoverageWeight() << ":" << std::endl;
#endif

                // If the newly added-together weight matches a previous weight we can combine
                // yet again, but this time we need to delete one of the segments.  This
                // can get rid of the first-segment partials that can't be otherwise collapsed:
                partial_segment_indices[i] = -1; // so we don't match the same one again
                const int matched_partial_index = findBestMatch2(*this,
                                                                partial_segment_indices,
                                                                new_pixel,
                                                                new_pixel.channels,
                                                                SpMask8::fullCoverage,
                                                                new_segment.flags(),
                                                                bin_count_sum,
                                                                color_threshold);
                // If matched again, combine them into this new match by ORing the spmasks.
                // Then delete the currently matched segment, and removing it from the list
                // of active matches:
                if (matched_partial_index >= 0)
                {
#ifdef DCX_DEBUG_COLLAPSING
                    if (debug_enable) std::cout << " - SECONDARY PARTIAL MATCH=" << matched_partial_index;
#endif
                    DeepSegment& combined_segment = getSegment(matched_partial_index);
                    combined_segment.metadata.spmask |= matched_bits;
                    removeSegment(matched_segment_index);
                }
                else
                {
                    // Restore the matched index if no match and save combined:
                    partial_segment_indices[i] = matched_segment_index;
                    matched_segment = new_segment;
                    matched_pixel = new_pixel;
                }

#ifdef DCX_DEBUG_COLLAPSING
                if (debug_enable) std::cout << " -> partial:" << std::endl;
#endif

            }

            // Turn off matched bits:
            in_segment_bits &= ~matched_bits;
        }

    } // for nPartialZmatches


    // Any remaining unmatched bits go to a new partial segment:
    if (in_segment_bits != SpMask8::zeroCoverage)
    {
        new_segment = in_segment;
        new_segment.metadata.spmask = in_segment_bits;
        append(new_segment, in_pixel);
#ifdef DCX_DEBUG_COLLAPSING
        if (debug_enable) {
            std::cout << "        UNMATCHED BITS - new partial segment:" << std::endl;
            new_segment.spMask().printPattern(std::cout, "        ");
        }
#endif
    }
#ifdef DCX_DEBUG_COLLAPSING
    if (debug_enable) std::cout << "        ---------------------------------------------------------" << std::endl;
#endif

}


//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

//
// Build a list of SegmentEdges from the list of DeepSegments
//

size_t
DeepPixel::buildSegmentEdges (const SpMask8& spmask,
                              SegmentEdgeList& segment_edges)
{
    segment_edges.clear();
    const bool useSpMasks = (!isLegacyDeepPixel() && !allFullCoverage());
    const uint32_t nSegments = m_segments.size();
#ifdef DCX_DEBUG_FLATTENER
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable) {
        std::cout << "      DeepPixel::buildSegmentEdges() sp_mask=" << std::hex << spmask << std::dec;
        std::cout << ", nSegments=" << nSegments << std::endl;
    }
#endif
    segment_edges.reserve(nSegments);
    for (uint32_t j=0; j < nSegments; ++j)
    {
        const DeepSegment& segment = m_segments[j];

        // Skip segment if not in spmask:
        if (useSpMasks && !segment.maskBitsEnabled(spmask))
            continue;

#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout.precision(8);
            std::cout << "        " << j << std::fixed << ": Zf=" << segment.Zf << " Zb=" << segment.Zb;
            std::cout << " flags=["; segment.printFlags(std::cout); std::cout << "]";
            std::cout << ", mask=" << std::hex << segment.spMask() << std::dec;
            std::cout << " segment_thickness=" << (segment.Zb - segment.Zf) << std::endl;
            if (useSpMasks)
                segment.spMask().printPattern(std::cout, "          ");
        }
#endif

        if (segment.isThin())
            segment_edges.push_back(DeepSegment::Edge(segment.Zf, j, DeepSegment::Edge::THIN ));

        else
        {
            segment_edges.push_back(DeepSegment::Edge(segment.Zf, j, DeepSegment::Edge::FRONT));
            segment_edges.push_back(DeepSegment::Edge(segment.Zb, j, DeepSegment::Edge::BACK ));
        }
    }

    return segment_edges.size();
}

//-------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------

//
// Log-merge together a subsection from a set of segments.
// Merge ALL-LOG no-spmasks samples - calculate total absorption for merged
// segments using Foundry-compatible (legacy Nuke) math.
//
// Segment 'subsegment' contains the Zf-Zb depth range to merge
// which must fall within the min/max depth range of the segment
// set - no checks are performed to verify this!
//

void
DeepPixel::mergeSectionLegacy (const SegmentEdgeSet& segment_set,
                               double Zf,
                               double Zb,
                               const ChannelSet& channels,
                               Pixelf& out,
                               DeepMetadata& merged_metadata)
{
    Pixelf subsection_color(channels);
    const double subsection_thickness = (Zb - Zf);

    out.erase(channels);
    double absorption_accum = 0.0;
    float merged_alpha = 0.0f;

    merged_metadata.spmask = SpMask8::fullCoverage;
    merged_metadata.flags  = DeepFlags::ALL_BITS_OFF;
    bool merged_all_matte = true;

#ifdef DCX_DEBUG_FLATTENER
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
#endif

    for (SegmentEdgeSet::const_iterator it=segment_set.begin(); it != segment_set.end(); ++it)
    {
#ifdef DCX_DEBUG_FLATTENER
        assert(*it < m_segments.size());
#endif
        const DeepSegment& segment = m_segments[*it];
        const Pixelf& segment_color = getSegmentPixel(segment);

        // Get subsection within segment:
        const double segment_thickness = (double(segment.Zb) - double(segment.Zf));
        const double t = (segment_thickness < EPSILONd)?1.0:(subsection_thickness / segment_thickness);

#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout.precision(8);
            std::cout << "          " << *it << " [" << segment.Zf << " " << segment.Zb;
            std::cout << " (" << segment_thickness << " thick)]";
            std::cout << "] t=" << t << ", LOG INTERPOLATE (LEGACY)";
        }
#endif

        // Only apply alpha correction if alpha is a grey value:
        const float interp_alpha = segment_color[Chan_A];
        subsection_color = segment_color;
        if (segment.isMatte())
        {
            // Matte object, blacken color channels except alpha:
            const float a = subsection_color[Chan_A];
            subsection_color.erase(channels);
            subsection_color[Chan_A] = a;
#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) std::cout << ", MATTE";
#endif

        }
        else
        {
            merged_all_matte = false;
            subsection_color[Chan_ACutout] = subsection_color[Chan_A];
        }

        // Composite with final alpha for all samples at this segment:
        if (interp_alpha <= 0.0f || interp_alpha >= 1.0f)
        {
            merged_alpha = merged_alpha*(1.0f - interp_alpha) + interp_alpha;
#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) std::cout << ", interp_alpha=" << interp_alpha << std::endl;
#endif
        }
        else
        {
            const double viz = 1.0 - CLAMP(interp_alpha);
            const float subsection_alpha = float(1.0 - pow(viz, t));
            const float correction = subsection_alpha / interp_alpha;
            foreach_channel(z, channels)
                subsection_color[z] *= correction;
            subsection_color[Chan_A] = subsection_alpha;

            merged_alpha = merged_alpha*(1.0f - subsection_alpha) + subsection_alpha;
#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) std::cout << ", interp_alpha=" << interp_alpha << ", subsection_alpha=" << subsection_alpha << std::endl;
#endif
        }

        const float subsection_alpha = subsection_color[Chan_A];
        if (subsection_alpha <= 0.0f)
        {
            // Alpha transparent, skip it

        }
        else if (subsection_alpha < 1.0f)
        {
            // Partially-transparent, find the absorbance-weighted average of the
            // unpremultiplied subsection color:
            const double absorbance = -log(double(1.0f - subsection_alpha));
            const float inv_subsection_alpha = 1.0f / subsection_alpha;
            foreach_channel(z, channels)
                out[z] += float(double(subsection_color[z]*inv_subsection_alpha) * absorbance);
            absorption_accum += absorbance;
        }
        else
        {
            // Alpha saturated, max absorbance:
            if (1/*segment_set.size() == 1*/)
            {
                // Handle single segment merges:
                foreach_channel(z, channels)
                    out[z] += subsection_color[z];
            }
            absorption_accum += MAX_ABSORBANCE;
        }

#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout.precision(6);
            std::cout << "             subsection_color" << subsection_color << std::endl;
            std::cout << "                 merged_color" << out << std::endl;
        }
#endif

    } // active segments merge loop

    // Weight final merged result by the accumulated absorption factor:
    if (absorption_accum < EPSILONd)
    {
        foreach_channel(z, channels)
            out[z] = 0.0f;
    }
    else
    {
        absorption_accum = 1.0 / absorption_accum;
        foreach_channel(z, channels)
            out[z] = float(double(out[z]) * absorption_accum)*merged_alpha;
    }
    out[Chan_A] = merged_alpha;
    out[Chan_SpCoverage] = 1.0f; // legacy segments are always have full spcoverage

    if (merged_all_matte)
        merged_metadata.flags |= DeepFlags::MATTE_OBJECT;

#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable) std::cout << "           final merged_color" << out << std::endl;
#endif

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
#ifdef DCX_DEBUG_FLATTENER
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable) {
        std::cout << "    DeepPixel::flatten() channels=" << out_channels << ", hasOverlaps=" << m_overlaps;
        std::cout << ", allFullCoverage=" << allFullCoverage() << ", isLegacy=" << isLegacyDeepPixel();
        std::cout << ", ANDmask=" << std::hex << m_accum_and_mask << ", ANDflags=" << m_accum_and_flags << std::dec;
    }
#endif

    // If full coverage or there's no spmasks than we just have to flatten once:
    if (allFullCoverage() || isLegacyDeepPixel() || interpolation == INTERP_OFF)
    {
#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable)
            std::cout << ": FULL-COVERAGE: interpolation=" << interpolationModeString(interpolation) << std::endl;
#endif
        if (!hasOverlaps() || interpolation == INTERP_OFF)
            flattenNoOverlaps(out_channels, out, SpMask8::fullCoverage); // No overlaps, do a simple linear flatten
        else
            flattenOverlapping(out_channels, out, SpMask8::fullCoverage, interpolation); // Merge overlapping deep segments
        return;
    }

    //================================================================================
    // SUBPIXEL FLATTENING LOOP
    // Flatten each subpixel individually where each subpixel may contain a different
    // combination of enabled segments. 
    //================================================================================
#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable) std::cout << " - SUBPIXEL-LOOP: interpolation=" << interpolationModeString(interpolation) << std::endl;
#endif

    out.erase(out_channels);
    out[Chan_ZFront] = INFINITYf;
    out[Chan_ZBack ] = INFINITYf;

    Pixelf flattened(out_channels);

#ifdef DCX_USE_LARGEST_COVERAGE_Z
    // Handle the Z accumulation with coverage weighting - i.e. pick Z from
    // the nearest sample with the largest coverage.
    // TODO: Finishing this code will likely produce more accurate flattened Z's
    std::map<float, int> coverage_map;
#endif

    // Flatten just the segments that has their sp mask bit enabled for the
    // current subpixel:
    SpMask8 sp_mask(1ull);
    for (int sp_bin=0; sp_bin < SpMask8::numBits; ++sp_bin, ++sp_mask) {
        if (!hasOverlaps(sp_mask, false/*force*/))
            flattenNoOverlaps(out_channels, flattened, sp_mask); // No overlaps, do a simple linear flatten
        else
            flattenOverlapping(out_channels, flattened, sp_mask, interpolation); // Merge overlapping deep segments

        if (flattened[Chan_A] < EPSILONf)
            continue;

        // Add the flattened subpixel color to accumulation pixel and handle Z's:
        out += flattened;
#ifdef DCX_USE_LARGEST_COVERAGE_Z
        // Record the ZFronts in a map to count the dominant Zf:
        const float Zf = flattened[Chan_ZFront];
        if (Zf < INFINITYf)
        {
            std::map<float, int>::iterator it = coverage_map.find(Zf);
            if (it == coverage_map.end())
                coverage_map[Zf] = 1;
            else
                ++it->second;
        }
#else
        // Find nearest Z's:
        out[Chan_ZFront] = std::min(out[Chan_ZFront], flattened[Chan_ZFront]);
#endif
        out[Chan_ZBack] = std::max(out[Chan_ZBack], flattened[Chan_ZBack]);
    }
    out /= float(SpMask8::width*SpMask8::height);

#ifdef DCX_USE_LARGEST_COVERAGE_Z
    // Find frontmost Z with largest coverage count:
    int max_coverage = 0;
    for (std::map<float, int>::iterator it=coverage_map.begin(); it != coverage_map.end(); ++it)
    {
        if (it->second > max_coverage)
        {
            out[Chan_ZFront] = std::min(out[Chan_ZFront], it->first);
            max_coverage = it->second;
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
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable) {
        std::cout << "   DeepPixel::flattenSubpixels() channels=" << out_channels;
        std::cout << ", interpolation=" << interpolationModeString(interpolation);
        std::cout << ", sp_mask=" << std::hex << spmask << std::dec << ", hasOverlaps=" << has_overlaps;
        std::cout << ", allFullCoverage=" << allFullCoverage() << ", isLegacy=" << isLegacyDeepPixel();
        std::cout << ", ANDmask=" << std::hex << m_accum_and_mask << ", ANDflags=" << m_accum_and_flags << std::dec;
        std::cout << std::endl;
    }
#endif

    // Are there overlaps...?:
    if (!has_overlaps || interpolation == INTERP_OFF)
        // No overlaps, do a simple linear flatten:
        flattenNoOverlaps(out_channels, out, spmask);
    else
        // Merge overlapping deep segments:
        flattenOverlapping(out_channels, out, spmask, interpolation);
}

//-------------------------------------------------------------------------------------

//
//  Flatten a list of segments in near-to-far order.
//

void
DeepPixel::flattenNoOverlaps (const ChannelSet& out_channels,
                              Pixelf& out,
                              const SpMask8& spmask)
{
    out.erase(out_channels);
    // Always fill in these output channels even though they may not be enabled
    // in the output pixel's channel set:
    out[Chan_A         ] = 0.0f;
    out[Chan_ZFront    ] = INFINITYf;
    out[Chan_ZBack     ] =-INFINITYf;
    out[Chan_ACutout   ] = 0.0f;
    // Inverse B-alpha (accumulated visibility value) and accumulated subpixel coverage
    // weight - both updated at each non-partial sample:
    out[Chan_Visibility] = 1.0f;
    out[Chan_SpCoverage] = 0.0f;

    const size_t nSegments = m_segments.size();
#ifdef DCX_DEBUG_FLATTENER
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable) {
        std::cout << "    DeepPixel::flattenNoOverlaps() channels=" << out_channels << ", nSegments=" << nSegments;
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
    comp_channels &= m_channels;        // only bother compositing requested output channels
    comp_channels += Mask_Opacities;    // but, always composite alpha & spcoverage even if not requested
    comp_channels -= Mask_Depths;       // don't composite depth channels, we handle those separately
    comp_channels -= Mask_Accumulators; // don't composite accumulator channels, we handle those separately
    //
    ChannelSet comp_channels_plus_cutouts(comp_channels);
    comp_channels_plus_cutouts += Chan_ACutout;
    //comp_channels_plus_cutouts += Chan_ARCutout;
    //comp_channels_plus_cutouts += Chan_AGCutout;
    //comp_channels_plus_cutouts += Chan_ABCutout;

    // Is this deep pixel full of legacy (no spmask, no interpolation flag) samples?
    const bool useSpMasks = (!isLegacyDeepPixel() && !allFullCoverage());

#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable) {
        std::cout << "      segments:" << std::endl;
        std::cout.precision(8);
        std::cout << std::fixed;
        for (size_t i=0; i < nSegments; ++i) {
            const DeepSegment& segment = m_segments[i];
            // Skip segment if not in active spmask:
            if (useSpMasks && !segment.maskBitsEnabled(spmask))
                continue;
            std::cout << "        " << i << ": Zf=" << segment.Zf << ", Zb=" << segment.Zb;
            std::cout << ", flags=["; segment.printFlags(std::cout); std::cout << "]";
            std::cout << ", spMask=" << std::hex << segment.spMask() << std::dec;
            std::cout << ", color" << getSegmentPixel(segment);
            std::cout << std::endl;
        }
    }
#endif

    // Put them in front to back order then composite each segment:
    this->sort();
    for (size_t i=0; i < nSegments; ++i)
    {
        const DeepSegment& segment = m_segments[i];
        // Skip segment if not in active spmask:
        if (useSpMasks && !segment.maskBitsEnabled(spmask))
            continue;
#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout << "     " << i << std::fixed << ": Zf=" << segment.Zf << " Zb=" << segment.Zb;
            std::cout << " flags=["; segment.printFlags(std::cout); std::cout << "]";
            std::cout << ", viz=" << out[Chan_Visibility];
        }
#endif

        compositeSegmentUnder(out, segment, comp_channels_plus_cutouts);
#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout << std::endl;
            if (useSpMasks)
                segment.spMask().printPattern(std::cout, "        ");
            const Pixelf& color = getSegmentPixel(segment);
            std::cout << "         COLOR" << color << std::endl;
            std::cout << "           "; out.print(std::cout, "OUT", 12/*prec*/, comp_channels_plus_cutouts);
            std::cout << ", viz=" << out[Chan_Visibility];
            std::cout << std::endl;
        }
#endif

        if (out[Chan_A] >= (1.0f - EPSILONf))
            break; // alpha saturated, all done

    } // nSegments

    // Ensure ZBack is valid:
    if (out[Chan_ZBack] < 0.0f)
        out[Chan_ZBack] = INFINITYf;

    // Final alpha is cutout-alpha channel:
    out[Chan_A] = (out[Chan_ACutout] >= (1.0f - EPSILONf))?1.0f:out[Chan_ACutout];

} // DeepPixel::flattenNoOverlaps


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


//
//  Flatten a list of segments in near-to-far order with overlap handling.
//

void
DeepPixel::flattenOverlapping (const ChannelSet& out_channels, Pixelf& out,
                               const SpMask8& spmask,
                               InterpolationMode interpolation)
{
    out.erase(out_channels);
    // Always fill in these output channels even though they may not be enabled
    // in the output pixel's channel set:
    out[Chan_A         ] = 0.0f;
    out[Chan_ZFront    ] = INFINITYf;
    out[Chan_ZBack     ] =-INFINITYf;
    out[Chan_ACutout   ] = 0.0f;
    // Inverse B-alpha (accumulated visibility value) and accumulated subpixel coverage
    // weight - both updated at each non-partial sample:
    out[Chan_Visibility] = 1.0f;
    out[Chan_SpCoverage] = 0.0f;

    const uint32_t nSegments = m_segments.size();
#ifdef DCX_DEBUG_FLATTENER
    const bool debug_enable = SAMPLINGXY(m_x, m_y);
    if (debug_enable) {
        std::cout << "    DeepPixel::flattenOverlapping() channels=" << out_channels;
        std::cout << ", interpolation=" << interpolationModeString(interpolation);
        std::cout << ", nSegments=" << nSegments << ", sp_mask=" << std::hex << spmask << std::dec;
    }
#endif
    if (nSegments == 0)
    {
        out[Chan_ZBack] = INFINITYf;
#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable)
        std::cout << " - no segments to flatten" << std::endl;
#endif
        return; // shouldn't happen...
    }

    // Get the channel set to composite:
    ChannelSet comp_channels(out_channels);
    comp_channels &= m_channels;        // only bother compositing requested output channels
    comp_channels += Mask_Opacities;    // but, always composite alpha & spcoverage even if not requested
    comp_channels -= Mask_Depths;       // don't composite depth channels, we handle those separately
    comp_channels -= Mask_Accumulators; // don't composite accumulator channels, we handle those separately
    //
    ChannelSet comp_channels_plus_cutouts(comp_channels);
    comp_channels_plus_cutouts += Chan_ACutout;
    //comp_channels_plus_cutouts += Chan_ARCutout;
    //comp_channels_plus_cutouts += Chan_AGCutout;
    //comp_channels_plus_cutouts += Chan_ABCutout;
    //
    // For matte-cutout erase loops:
    ChannelSet comp_channels_no_opacities_plus_cutouts(comp_channels_plus_cutouts);
    comp_channels_no_opacities_plus_cutouts -= Mask_Opacities;

    // Force log interpolation in legacy mode:
    const bool legacySamples = isLegacyDeepPixel();
    if (legacySamples)
        interpolation = INTERP_LOG;

    // Is this deep pixel full of legacy (no spmask, no interpolation flag) samples?
    const bool useSpMasks = (!legacySamples && !allFullCoverage());

#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable) {
        std::cout << ", allZeroCoverage=" << allZeroCoverage() << ", allFullCoverage=" << allFullCoverage();
        std::cout << ", allVolumetric=" << allVolumetric();
        std::cout << ", useSpMasks=" << useSpMasks << ", interpolation=" << interpolationModeString(interpolation) << std::endl;
    }
#endif

    // Build a list of segment 'edges' that marks the start or end of each segment:
    SegmentEdgeList segment_edges;
    const uint32_t nEdges = buildSegmentEdges(spmask, segment_edges);
    if (nEdges == 0)
    {
        out[Chan_ZBack] = INFINITYf;
#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable)
        std::cout << " - no segment edges to flatten" << std::endl;
#endif
        return; // No valid segments to combine - shouldn't happen!
    }

    // Z-sort the edges - this will also change the order based on edge type:
    std::sort(segment_edges.begin(), segment_edges.end());

    // As we step front to back through the edges we enable or disable that
    // edge in a set of active edge segments. Each active segment is subdivided
    // for that subsection of z-range and and merged with the other active
    // edges forming a final segment that can be accumulated normally:
    SegmentEdgeSet active_segments;
    int nLogSamples = 0;
    int nLinSamples = 0;

    Pixelf  segment_color(comp_channels_plus_cutouts);
    Pixelf  section_color(comp_channels_plus_cutouts);
    Pixelf   merged_color(comp_channels_plus_cutouts);
    Pixelf additive_color(comp_channels_plus_cutouts);

#ifdef DCX_DEBUG_FLATTENER
    if (debug_enable) {
        std::cout << "      segment_edges:" << std::endl;
        std::cout.precision(8);
        for (uint32_t j=0; j < nEdges; ++j) {
            const DeepSegment::Edge& edge = segment_edges[j];
            const DeepSegment& segment = m_segments[edge.segment];
            const Pixelf& pixel = getSegmentPixel(edge.segment);
            std::cout.precision(8);
            std::cout << "       " << ((j < 100)?((j < 10)?"  ":" "):"") << j;
            std::cout << ": segment=" << ((edge.segment < 100)?((edge.segment < 10)?"  ":" "):"") << edge.segment;
            std::cout << "[" << ((edge.type == DeepSegment::Edge::FRONT)?"front":(edge.type == DeepSegment::Edge::BACK)?" back":" thin");
            std::cout << "] depth=" << edge.depth;
            std::cout << ", Zf=" << segment.Zf << " Zb=" << segment.Zb;
            std::cout << " flags=["; segment.printFlags(std::cout); std::cout << "]";
            if (edge.type != DeepSegment::Edge::BACK) {
                std::cout << " segment_thickness=" << (segment.Zb - segment.Zf);
                std::cout << ", color" << pixel;
            }
            std::cout << std::endl;
        }
        std::cout << "      ----------------- edges loop -------------------" << std::endl;
        std::cout << "        ========= "; out.print(std::cout, "OUT", 12/*prec*/, comp_channels_plus_cutouts);
        std::cout << ", viz=" << out[Chan_Visibility];
        std::cout << " =========" << std::endl;
    }
#endif
    for (uint32_t j=0; j < nEdges; ++j)
    {
        const DeepSegment::Edge& edge = segment_edges[j];
#ifdef DCX_DEBUG_FLATTENER
        assert(edge.segment < nSegments);
#endif
        const DeepSegment& segment0 = m_segments[edge.segment];

#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout.precision(8);
            std::cout << "      e" << j << ": segment=" << edge.segment;
            std::cout << ", ";
            if (edge.type == DeepSegment::Edge::FRONT)
                std::cout << "front - ** start edge **";
            else if (edge.type == DeepSegment::Edge::BACK)
                std::cout << "back - ** remove edge **";
            else
                std::cout << "thin - ** UNDER/PLUS immediate **";
            if (edge.type != DeepSegment::Edge::BACK) {
                std::cout << " flags=["; segment0.printFlags(std::cout);
                std::cout << "] depth=" << edge.depth;
                std::cout << ", color" << getSegmentPixel(edge.segment);
            }
            std::cout << std::endl;
        }
#endif

        bool compositeImmediate = false;

        // We need to keep track of which segment have started but not yet finished.
        // So, for each edge add and remove samples as appropriate:
        if (edge.type == DeepSegment::Edge::FRONT)
        {
            if (segment0.isHardSurface())
                ++nLinSamples;
            else
                ++nLogSamples;
            // Check if there's no other active segments and the next
            // edge is this one's back.  If so we don't need to bother
            // combining and can composite immediately:
            if (active_segments.empty() && j < (nEdges - 1) &&
                segment_edges[j+1].segment == edge.segment && segment_edges[j+1].type == DeepSegment::Edge::BACK)
                compositeImmediate = true;
            else
                active_segments.insert(edge.segment); // Add samples on their front edge
        }
        else if (edge.type == DeepSegment::Edge::BACK)
        {
            active_segments.erase(edge.segment); // Remove samples on their back edge
            if (segment0.isHardSurface())
                --nLinSamples;
            else
                --nLogSamples;
        }
        else
        {
            // edge.type == DeepSegment::Edge::THIN
            // For samples where front == back, don't bother attempting to merge with
            // other active segments since it has no thickness - immediately composite it:
            compositeImmediate = true;
        }

        if (compositeImmediate)
        {
#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) {
                getSegmentPixelWithMetadata(segment0, comp_channels_plus_cutouts, segment_color);
                std::cout << "                 color" << segment_color << " - COMPOSITE IMMEDIATE" << std::endl;
            }
#endif

            compositeSegmentUnder(out, segment0, comp_channels);
#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) {
                std::cout << "        ========= "; out.print(std::cout, "OUT", 12/*prec*/, comp_channels_plus_cutouts);
                std::cout << ", viz=" << out[Chan_Visibility];
                std::cout << " =========" << std::endl;
            }
#endif

            // No need to composite further segments if solid alpha:
            if (out[Chan_A] >= (1.0f - EPSILONf))
            {
                out[Chan_A] = (out[Chan_ACutout] >= (1.0f - EPSILONf))?1.0f:out[Chan_ACutout];
                return;
            }
        }

        // No active edges?  Skip to next edge:
        if (active_segments.empty())
            continue;

        // There is always at least one BACK edge in the list at this point to disable
        // each active FRONT edge. In the final iteration in the iterator for-loop,
        // active_segments should be empty!
#ifdef DCX_DEBUG_FLATTENER
        assert((j + 1) < nEdges); // shouldn't heppen...
#endif

        bool linear_interp0 = (interpolation == INTERP_LIN ||
                              (interpolation == INTERP_AUTO && segment0.isHardSurface()));

        // Get Z distance between this edge and the next edge and build
        // a temp segment that spans the subsection:
        DeepSegment subsegment(edge.depth, segment_edges[j + 1].depth, segment0.index, segment0.metadata);
        const double subsegmentZf = double(subsegment.Zf);
        const double subsegmentZb = double(subsegment.Zb);
        const double subsegment_thickness = (subsegmentZb - subsegmentZf);

        // If segment too thin to interpolate - skip it:
        if (!linear_interp0 && subsegment.thickness() < EPSILONf)
            continue;


#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout << "        combine active samples: interpolation=" << interpolationModeString(interpolation);
            std::cout << ", nLogSamples=" << nLogSamples << ", nLinSamples=" << nLinSamples;
            std::cout << ", [ ";
            for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                std::cout << *it << ", ";
            std::cout.precision(8);
            std::cout << "], Zf=" << subsegment.Zf << ", Zb=" << subsegment.Zb << ", thickness=" << subsegment.thickness();
            std::cout << std::endl;
        }
#endif

        //---------------------------------------------------------------------------------
        // Handle combinations of log and lin segment interpolation types.
        //
        // If we have all-log active samples and no linear active samples then we can
        // use Florian's log merge math, otherwise we need to step through the
        // sub-segment range evaluating the samples at each step distance and under them.
        //
        // TODO: can the all-lin case be done through simple math?
        //
        //---------------------------------------------------------------------------------


#ifdef DCX_DEBUG_FLATTENER
        assert(*active_segments.begin() < m_segments.size());
#endif

        if (interpolation == INTERP_LOG ||
            (interpolation == INTERP_AUTO && nLogSamples > 0 && nLinSamples == 0))
        {
            //=========================================================================================
            //
            // ALL-LOG samples to merge
            //
            //=========================================================================================

            // Handle legacy vs. spmask log deep samples:
            if (legacySamples && !useSpMasks)
            {
                //---------------------------------------------------------------
                // No subpixel masks, use the legacy (old Nuke) log merging math:
                //---------------------------------------------------------------
                DeepMetadata merged_metadata;
                mergeSectionLegacy(active_segments,
                                   subsegment.Zf,
                                   subsegment.Zb,
                                   comp_channels_plus_cutouts,
                                   merged_color,
                                   merged_metadata);

                compositeSegmentUnder(out, subsegment, merged_color, comp_channels);
            }
            else
            {
                //-----------------------------------------------------------------
                // Log-merge together a subsection from the active set of segments
                // using volumetric log merging math from Florian's deep doc:
                //-----------------------------------------------------------------

                int merged_count = 0;
                int additive_count = 0;
                additive_color.erase(comp_channels_plus_cutouts);
                merged_color.erase(comp_channels_plus_cutouts);
                merged_color[Chan_Visibility] = 1.0f;

                for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                {
#ifdef DCX_DEBUG_FLATTENER
                    assert(*it < m_segments.size());
#endif
                    const DeepSegment& active_segment = m_segments[*it];
                    getSegmentPixelWithMetadata(active_segment, comp_channels_plus_cutouts, segment_color);

                    const double segment_thickness = (double(active_segment.Zb) - double(active_segment.Zf));
                    const double t = (segment_thickness < EPSILONd)?1.0:(subsegment_thickness / segment_thickness);
#ifdef DCX_DEBUG_FLATTENER
                    if (debug_enable) {
                        const std::streamsize prec = std::cout.precision();
                        std::cout.precision(8);
                        std::cout << "          " << *it << " [" << active_segment.Zf << " " << active_segment.Zb;
                        std::cout << " (" << segment_thickness << " thick)]";
                        std::cout << ", t=" << t << ", LOG INTERPOLATE";
                        //std::cout << ", channels=" << channels;
                        std::cout << " color" << segment_color;
                        std::cout << std::endl;
                        std::cout.precision(prec);
                    }
#endif

                    if (active_segment.isAdditive() || active_segment.hasPartialSpCoverage())
                    {
                        // If additive do linear interpolation:
                        if (active_segment.isMatte())
                        {
                            // Matte object - blacken the color channels but interpolate alpha and spcoverage:
                            section_color.erase(comp_channels_plus_cutouts);
                            DeepSegment::interpolateLin(segment_color, t, Mask_Opacities, section_color);
                        }
                        else
                        {
                            DeepSegment::interpolateLin(segment_color, t, comp_channels_plus_cutouts, section_color);
                            section_color[Chan_ACutout] = section_color[Chan_A];
                        }
                        ++additive_count;
                        additive_color += section_color;
                        if (additive_color[Chan_SpCoverage] >= (1.0f - EPSILONf))
                        {
                            DeepSegment::mergeLog(additive_color, comp_channels_plus_cutouts, merged_color);
                            additive_color.erase(comp_channels_plus_cutouts);
                        }
                    }
                    else
                    {
                        // Do log interpolation:
                        if (active_segment.isMatte())
                        {
                            // Matte object - blacken the color channels but interpolate alpha and spcoverage:
                            section_color.erase(comp_channels_plus_cutouts);
                            DeepSegment::interpolateLog(segment_color, t, Mask_Opacities, section_color);
                        }
                        else
                        {
                            // Do volumetric log merge (merge math from Florian's deep doc):
                            DeepSegment::interpolateLog(segment_color, t, comp_channels_plus_cutouts, section_color);
                            section_color[Chan_ACutout] = section_color[Chan_A];
                        }
                        section_color[Chan_SpCoverage] = 1.0f;
                        DeepSegment::mergeLog(section_color, comp_channels_plus_cutouts, merged_color);
                        ++merged_count;
                    }
#ifdef DCX_DEBUG_FLATTENER
                    if (debug_enable) {
                        std::cout << "                section_color" << section_color << std::endl;
                        std::cout << "               additive_color" << additive_color << std::endl;
                        std::cout << "                 merged_color" << merged_color << std::endl;
                    }
#endif

                } // active segments loop

                if (merged_count > 0)
                {
                    compositeSegmentUnder(out,
                                          subsegment.Zf, subsegment.Zb,
                                          DeepFlags(0x0)/*non-additive*/,
                                          merged_color,
                                          comp_channels);
                }

                if (additive_count > 0)
                {
                    compositeSegmentUnder(out,
                                          subsegment.Zf, subsegment.Zb,
                                          DeepFlags(DeepFlags::ADDITIVE),
                                          additive_color,
                                          comp_channels);
                }

            } // legacySamples && useSpMasks

        }
        else if (nLogSamples == 0 && nLinSamples == 1)
        {
            //-----------------------------------------------------
            // Only 1 lin sample?  Extract it and composite.
            //-----------------------------------------------------
            const DeepSegment& active_segment = m_segments[*active_segments.begin()];

            extractSubSectionLin(active_segment, subsegment, comp_channels_plus_cutouts, section_color);
#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) std::cout << "         section_color" << section_color << std::endl;
#endif
            compositeSegmentUnder(out, active_segment, section_color, comp_channels);

        }
#if 0
        else if (nLogSamples == 0 && nLinSamples > 1)
        {
            //-----------------------------------------------------------------------------------------
            // All-lin: (TODO: finish implementation, if math is possible!)
            //-----------------------------------------------------------------------------------------
            //
            //DeepMetadata merged_metadata;
            //mergeSubSectionLin(subsegment,
            //                   active_segments,
            //                   comp_channels_plus_cutouts,
            //                   merged_color,
            //                   merged_metadata);
            //compositeSegmentUnder(out, segment, merged_color, comp_channels);
        }
#endif
        else
        {
            //-----------------------------------------------------------------------------------------
            // Mixture of lin and log samples.
            // Step through sub-segment sampling and UNDER-ing to find the
            // final merged value.
            //-----------------------------------------------------------------------------------------

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
            const double step_size = subsegment_thickness / double(SEGMENT_SAMPLE_STEPS - 1);
            //const double step_t = step_size / subsegment_thickness;

#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) {
                std::cout << "            --------------------    ";
                const double t = (subsegmentZf - segment0.Zf) / (segment0.Zb - segment0.Zf);
                std::cout << "t=" << t << ", subt=0.00000000, zoffset=0.00000000";
                std::cout << "    ---------------------" << std::endl;
                std::cout << "            0:            "; out.print(std::cout, "OUT", 12/*prec*/, comp_channels_plus_cutouts);
                std::cout.precision(3);
                std::cout << ", viz=" << out[Chan_Visibility] << std::endl;
            }
#endif

            // Step through the subsegment.Zf-subsegment.Zb range interpolating the active segments,
            // averaging them together to make one slice, then UNDER-ing the slice:
            double Zf_prev = subsegmentZf;
            for (uint32_t step=1; step < SEGMENT_SAMPLE_STEPS; ++step)
            {
                // Interpolate to step offset:
                const double Zf = Zf_prev;
                const double Zb = subsegmentZf + double(step)*step_size;
                Zf_prev = Zb;

                int merged_count = 0;
                merged_color.erase(comp_channels_plus_cutouts);
                float accum_alpha = 0.0f;
                //float merged_alpha = 0.0f;

                int additive_count = 0;
                additive_color.erase(comp_channels_plus_cutouts);

#ifdef DCX_DEBUG_FLATTENER
                if (debug_enable) {
                    std::cout << "            --------------------    ";
                    std::cout.precision(8);
                    const double t = (Zb - segment0.Zf) / (segment0.Zb - segment0.Zf);
                    const double subt = (Zb - subsegmentZf) / subsegment_thickness;
                    std::cout << "t=" << t << ", subt=" << subt << ", zoffset=" << double(step)*step_size << ", Zf=" << Zf << ", Zb=" << Zb;
                    std::cout << "    ---------------------" << std::endl;
                }
#endif

                for (SegmentEdgeSet::const_iterator it=active_segments.begin(); it != active_segments.end(); ++it)
                {
#ifdef DCX_DEBUG_FLATTENER
                    assert(*it < nSegments);
#endif
                    const DeepSegment& active_segment = m_segments[*it];

                    // Merge or add the segment? If the segment's marked additive then always
                    // linear-interpolate and add it to the additive accumulator.
                    // If it's not additive then interpolate depending on surface type and
                    // merge with the merging accumulator:
                    if (active_segment.isAdditive() || active_segment.hasPartialSpCoverage())
                    {
#ifdef DCX_DEBUG_FLATTENER
                        if (debug_enable) std::cout << "        PAR";
#endif
                        // Always linear-interpolate additive samples, even log ones:
                        extractSubSectionLin(active_segment, Zf, Zb, comp_channels_plus_cutouts, section_color);

                        ++additive_count;
                        additive_color += section_color;
                    }
                    else
                    {
                        // Get the subsection color depending on interpolation mode:
#ifdef DCX_DEBUG_FLATTENER
                        if (debug_enable) {
                            std::cout << "        ";
                            if (active_segment.isHardSurface()) std::cout << "LIN"; else std::cout << "LOG";
                        }
#endif
                        extractSubSection(active_segment, Zf, Zb, comp_channels_plus_cutouts, section_color);

                        ++merged_count;
                        merged_color += section_color * section_color[Chan_A];
                        accum_alpha += section_color[Chan_A];
                        //merged_alpha = merged_alpha*(1.0f - section_color[Chan_A]) + section_color[Chan_A];
                    }
#ifdef DCX_DEBUG_FLATTENER
                    if (debug_enable) {
                        std::cout << " " << step << ":  section_color" << section_color;
                        const double t = (Zb - active_segment.Zf) / (active_segment.Zb - active_segment.Zf);
                        std::cout << " t=" << t;
                        std::cout << ", segment=" << *it << ", flags="; active_segment.printFlags(std::cout);
                        std::cout << std::endl;
                    }
#endif

                } // active segments loop

                if (merged_count > 0)
                {
#if 1
                    if (accum_alpha > 0.0001f)
#else
                    if (accum_alpha > EPSILONf)
#endif
                        merged_color /= accum_alpha;
#ifdef DCX_DEBUG_FLATTENER
                    if (debug_enable) {
                        std::cout << "                 merged_color" << merged_color;
                        std::cout << ", accum_alpha=" << accum_alpha << std::endl;
                    }
#endif
                    compositeSegmentUnder(out,
                                          float(Zf), float(Zb),
                                          DeepFlags(0x0)/*non-additive*/,
                                          merged_color,
                                          comp_channels);
                }

                if (additive_count > 0)
                {
#ifdef DCX_DEBUG_FLATTENER
                    if (debug_enable) std::cout << "               additive_color" << additive_color << std::endl;
#endif
                    compositeSegmentUnder(out,
                                          float(Zf), float(Zb),
                                          DeepFlags(DeepFlags::ADDITIVE),
                                          additive_color,
                                          comp_channels);
                }

#ifdef DCX_DEBUG_FLATTENER
                if (debug_enable) {
                    std::cout << "                          "; out.print(std::cout, "OUT", 12/*prec*/, comp_channels_plus_cutouts);
                    std::cout << ", viz=" << out[Chan_Visibility] << std::endl;
                }
#endif

                // If final alpha is now saturated we're done:
                if (out[Chan_A] >= (1.0f - EPSILONf))
                {
                    out[Chan_A] = 1.0f;
                    break;
                }

            } // SEGMENT_SAMPLE_STEPS loop


        } // mixed log/lin && active-samples > 1


#ifdef DCX_DEBUG_FLATTENER
        if (debug_enable) {
            std::cout << "        ========= "; out.print(std::cout, "OUT", 12/*prec*/, comp_channels_plus_cutouts);
            std::cout << ", viz=" << out[Chan_Visibility];
            std::cout << " =========" << std::endl;
        }
#endif

        // If alpha is now saturated we're done:
        if (out[Chan_A] >= (1.0f - EPSILONf))
        {
#ifdef DCX_DEBUG_FLATTENER
            if (debug_enable) std::cout << "          saturated, stop" << std::endl;
            // Clear the active_segments list so the exit asserts don't fail:
            active_segments.clear();
            nLogSamples = nLinSamples = 0;
#endif
            break;
        }

    } // for nSegments loop

    // Ensure ZBack is valid:
    if (out[Chan_ZBack] < 0.0f)
        out[Chan_ZBack] = INFINITYf;

    // Final alpha is cutout-alpha channel:
    out[Chan_A] = (out[Chan_ACutout] >= (1.0f - EPSILONf))?1.0f:out[Chan_ACutout];

#ifdef DCX_DEBUG_FLATTENER
    assert(active_segments.empty());
    assert(nLogSamples == 0);
    assert(nLinSamples == 0);
#endif

} // DeepPixel::flattenOverlapping


//----------------------------------------------------------------------------

OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
