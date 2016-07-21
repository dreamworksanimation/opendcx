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
/// @file DcxDeepPixel.h

#ifndef INCLUDED_DCX_DEEPPIXEL_H
#define INCLUDED_DCX_DEEPPIXEL_H

//-----------------------------------------------------------------------------
//
//  struct  DeepMetadata
//  class   DeepSegment
//  class   DeepPixel
//
//-----------------------------------------------------------------------------

#include "DcxChannelSet.h"
#include "DcxChannelDefs.h"
#include "DcxPixel.h"
#include "DcxSpMask.h"

#include <iostream>
#include <vector>
#include <math.h> // for log1p, expm1


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//-------------------------------------------------------------------------------
//
//  struct DeepMetadata
// 
//      Stores extra information about a DeepSegment.
//      For the moment it only stores the subpixel mask and the flags, but this
//      could be made to hold arbitrary attributes.
// 
//      These values are packed & unpacked via float & half channels for file IO.
//
//      (TODO: how to support variable mask sizes?  Do we want to...?)
//      (TODO: support arbitrary attributes?)
//
//-------------------------------------------------------------------------------

struct DCX_EXPORT DeepMetadata
{


    SpMask8     spmask;         // Subpixel mask
    DeepFlag    flags;          // Flags - interpolation-type, matte-mode, etc.


    //-----------------------------------------
    // Default constructor leaves junk in vars.
    //-----------------------------------------

    DeepMetadata () {}


    //------------
    // Constructor
    //------------

    DeepMetadata (SpMask8 _mask,
                  DeepFlag _flags);


    //--------------------------------
    // Print the list of enabled flags
    //--------------------------------

    void    printFlags (std::ostream&) const;

};


//
// Default channel names for deep metadata IO
//

const char* const spMask8ChannelName1  = "spmask.1";
const char* const spMask8ChannelName2  = "spmask.2";
const char* const flagsChannelName     = "spmask.flags";



//-------------------------------------------------------------------------------------
//
//  class DeepSegment
//
//      A single deep sample describing a linear segment of Z-space where Zf <= Zb.
//
//      The color channel data describes the values at Zb, so finding values between
//      Zf and Zb requires linear or log interpolation depending on the
//      interpolation-type flag in the metadata.
//
//      Note that this class does not store the actual channel data so that the adding,
//      deleting, sorting, etc of DeepSegments is lightweight and fast.
//
//-------------------------------------------------------------------------------------

class DCX_EXPORT DeepSegment
{
  public:

    float           Zf, Zb;         // Z-front / Z-back depth positions
    int             index;          // Index into an array of channel data storage. -1 indicates non-assignment!
    DeepMetadata    metadata;       // Flags - interpolation-type, matte-mode, etc.


  public:
    //-----------------------------------------
    // Default constructor leaves junk in vars.
    //-----------------------------------------
    DeepSegment () {}


    //---------------------------------------
    // Constructor
    // If Zf > Zb, Zf is clamped to Zb.
    //---------------------------------------

    DeepSegment (float _Zf, float _Zb,
                 int _index = -1,
                 const DeepMetadata& _metadata = DeepMetadata(SpMask8::fullCoverage,
                                                              DEEP_EMPTY_FLAG));


    //--------------------------------------------
    // Set both Zf & Zb - checks for bad Z's and
    // tries to find reasonable solution...
    //--------------------------------------------

    void    setDepths (float _Zf, float _Zb);


    //-------------------------
    // Used by the sort routine
    //-------------------------

    bool operator < (const DeepSegment&) const;


    //-------------------------
    // DeepFlag metadata access
    //-------------------------

    DeepFlag    flags () const;

    //-----------------------------------------------------------------
    // Does the segment represent a solid (hard) or volumetric surface?
    // If hard-surface use linear interpolation between Zf/Zb.
    // If volumetric use log interpolation between Zf/Zb.
    //-----------------------------------------------------------------

    bool    isHardSurface () const;
    bool    isVolumetric () const;


    //-------------------------------------------------------
    // Is Z-depth of segment zero (thin) or non-zero (thick)?
    //-------------------------------------------------------

    bool    isThin () const;
    bool    isThick () const;


    //-------------------------------------------------------
    // Should the segment cutout (act as a matte) the segments behind it?
    //-------------------------------------------------------

    bool    isMatte () const;


    //-------------------------------------------------------------------
    // Sample should be added to surrounding samples rather than under-ed
    // (TODO: Is this required any more now that
    //  DEEP_PARTIAL_BIN_COVERAGE does the same thing?)
    //-------------------------------------------------------------------

    bool    isAdditive () const;


    //-----------------------------------------------------------------------------
    // Does the sample have subpixel coverage baked-in to color, alpha, etc values?
    // If so the sample is handled as additive when composited.
    // This normally indicates filtering or resampling of subpixel masks has been
    // applied.
    //-----------------------------------------------------------------------------

    bool    hasPartialSubpixelBinCoverage () const;


    //---------------------
    // Subpixel mask access
    //---------------------

    SpMask8&        spMask ();
    const SpMask8&  spMask () const;


    //---------------------------------------------------------------------------
    // Are all subpixel mask bits off or on?
    //
    // If all are OFF this usually indicates a 'legacy' deep sample containing no
    // subpixel mask data which is interpreted as volumetric.
    //
    // If all bits are ON this can simplify and speed up sample operations,
    // especially compositing (flattening) as iterating through subpixels is
    // uneccessary.
    //---------------------------------------------------------------------------

    bool    zeroCoverage () const;

    bool    fullCoverage () const;

    bool    hasSpMasks () const; // same as zeroCoverage()


    //------------------------------------------------------------------
    // Return true if specific bits are enabled in the subpixel mask.
    // Note that zero-bits-on is considered all-bits-on for these tests.
    //------------------------------------------------------------------

    bool    maskBitsEnabled (const SpMask8& check_bits) const;

    bool    maskBitEnabled (int bit_index) const;


    //--------------------------------------------------------------
    // Convert the subpixel mask to coverage weight.
    // This is the same as float(spMask8BitsOn(metadata.mask))/64.0f
    // Note that zero-bits-on is considered all-bits-on for this.
    //--------------------------------------------------------------

    float   getCoverage () const;


    //----------------------------------------------------------
    // Apply the subpixel mask weight (coverage) to the channels
    //----------------------------------------------------------

    void    applyMask (const ChannelSet& channels,
                       Pixelf& color);


    //-----------------------------------------------------------------
    // Sample interpolation
    // Do linear or log interpolation at position t (0-1) within Zf-Zb,
    // depending on interpolation-type flag.
    //
    // The input pixel represents the value at Zf.  Interpolation
    // is performed on the intersection of 'channels' and the
    // input pixel's ChannelSet.
    // 
    // The output pixel is a copy of the input pixel with changes
    // to only the interpolated channels.
    //-----------------------------------------------------------------

    void    interpolate (Pixelf& in,
                         const ChannelSet& do_channels,
                         float t,
                         Pixelf& out) const;


    //------------------------------------------------------------
    // Logarithmic sample interpolation
    // Do log interpolation at position t (0-1) within Zf-Zb.
    //
    // Note that this uses the density math from the OpenEXR deep
    // docs translated to ChannelSet-loop use.
    //
    // The input pixel represents the value at Zf.  Interpolation
    // is performed on the intersection of 'channels' and the
    // input pixel's ChannelSet.
    // 
    // The output pixel is a copy of the input pixel with changes
    // to only the interpolated channels.
    //
    // (TODO: move/add this to the Pixel class?)
    //------------------------------------------------------------

    static void  interpolateLog (const Pixelf& in,
                                 const ChannelSet& do_channels,
                                 float t,
                                 Pixelf& out);


    //-----------------------------------------------------------
    // Linear sample interpolation
    // Do linear interpolation at position t (0-1) within Zf-Zb.
    //
    // The input pixel represents the value at Zf.  Interpolation
    // is performed on the intersection of 'channels' and the
    // input pixel's ChannelSet.
    // 
    // The output pixel is a copy of the input pixel with changes
    // to only the interpolated channels.
    //
    // (TODO: move/add this to the Pixel class?)
    //-----------------------------------------------------------

    static void  interpolateLin (const Pixelf& in,
                                 const ChannelSet& do_channels,
                                 float t,
                                 Pixelf& out);


    //-----------------------------------------------------------
    // Print info about DeepSegment to output stream
    //-----------------------------------------------------------

    void    printInfo (std::ostream&) const;
    void    printFlags (std::ostream&) const;
    friend  std::ostream& operator << (std::ostream&,
                                       const DeepSegment&);

};



//----------------------------------------------------------------------------------------
//
//  class DeepPixel
//
//      Contains a series of DeepSegments and their associated Pixels (containing float
//      channel data,) which combined together comprise a series of deep samples.
//
//      Supports metadata storage for each deep sample and offers methods to aid the
//      manipulation of samples within the deep pixel, including compositing (flattening,)
//      and sorting.
//      Because a DeepSegment is lightweight the list can be rearranged and sorted very
//      quickly.  The list of large Pixel channel data structures is kept static.
//
//      TODO: add methods for removing DeepSegments and their Pixels.
//      TODO: investigate cost of using varying-sized Pixels
//      TODO: add methods to get interpolated value at specific depths
//      TODO: extend flatten methods to accept near/far depth range to flatten within
//
//----------------------------------------------------------------------------------------

class DCX_EXPORT DeepPixel
{
  public:

    //-----------------------------------------
    // Constructors
    //-----------------------------------------
    DeepPixel (const ChannelSet& channels);
    DeepPixel (const ChannelIdx z);
    DeepPixel (const DeepPixel& b);

    //---------------------------------------------------------
    // Read-only ChannelSet access
    //      This ChannelSet is shared between all DeepSegments.
    //---------------------------------------------------------

    const   ChannelSet& channels () const;


    //---------------------------------------------------------
    // Assign a ChannelSet
    //      This ChannelSet is shared between all DeepSegments.
    //---------------------------------------------------------

    void    setChannels (const ChannelSet& c);


    //-----------------------------------------------------
    // Empty the segment list and clear most shared values.
    // The shared ChannelSet is unaffected.
    //-----------------------------------------------------

    void    clear ();


    //----------------------------
    // DeepSegment list management
    //----------------------------

    bool    empty () const;
    size_t  size () const;
    size_t  capacity () const;
    void    reserve (size_t n);


    //---------------------------------------------------------
    // DeepSegment Handling
    //---------------------------------------------------------

    DeepSegment& operator [] (size_t segment);
    const DeepSegment& operator [] (size_t segment) const;

    DeepSegment& getSegment (size_t segment);
    const DeepSegment& getSegment (size_t segment) const;

    // Sort the segments.  If the sorted flag is already true this returns quickly.
    void    sort (bool force=false);

    // Return the index of the DeepSegment nearest to Z and inside the distance of Z +- maxDistance, or -1 if nothing found.
    int     nearestSegment (double Z, double maxDistance=0.0);

    // Check for overlaps between samples and return true if so.
    bool    hasOverlaps (const SpMask8& spmask=SpMask8::fullCoverage, bool force=false);

    // Returns true if at least one segment has spmasks (same as !allZeroCoverage())
    bool    hasSpMasks ();

    // Returns true if all segment spmasks are zero coverage and not hard-surface tagged.
    bool    isLegacyDeepPixel ();

    // Returns true if all segment spmasks are zero coverage - this may indicate a legacy deep pixel.
    bool    allZeroCoverage ();

    // Returns true if all segment spmasks are full coverage.
    bool    allFullCoverage ();

    // Returns true is all segments are volumetric (log interp.)
    bool    allVolumetric ();
    bool    anyVolumetric ();

    // Returns true is all segments are hard-surface (lin interp.)
    bool    allHardSurface ();
    bool    anyHardSurface ();

    // Returns true is all segments are matte.
    bool    allMatte ();
    bool    anyMatte ();

    // Asssigns a subpixel mask to a range of segments
    void    setSegmentMask (const SpMask8& mask, size_t start=0, size_t end=10000);

    // Add an empty DeepSegment to the end of the list, returning its index.
    size_t  append ();
    // Add a DeepSegment to the end of the list.
    size_t  append (const DeepSegment& bs);
    // Add a DeepSegment to the end of the list.
    size_t  append (const DeepSegment& bs, const Pixelf& bp);
    // Copy one segment from the second DeepPixel.
    size_t  append (const DeepPixel& b, size_t segment_index);
    // Combine all the segments of two DeepPixels.
    void    append (const DeepPixel& b);


    //---------------------------------------------------------
    // Arithmetic ops
    // Note that there are no *, /, -, + operators to avoid the
    // high cost of constructing & destroying DeepPixels.
    // Try to use these in-place modifiers.
    //---------------------------------------------------------

    DeepPixel& operator += (float val);
    DeepPixel& operator -= (float val);
    DeepPixel& operator *= (float val);
    DeepPixel& operator /= (float val);


    //------------------------------------
    // Read/Write DeepSegment Pixel access
    //------------------------------------

    const Pixelf&   getPixel (size_t pixel_index) const;
    Pixelf&         getPixel (size_t pixel_index);
    const Pixelf&   getSegmentPixel (size_t segment_index) const;
    Pixelf&         getSegmentPixel (size_t segment_index);
    const Pixelf&   getSegmentPixel (const DeepSegment& segment) const;
    Pixelf&         getSegmentPixel (const DeepSegment& segment);


    //--------------------------------------------------------------
    // Get a Pixel sampled at depth Z, possibly using interpolation.
    // Z is clamped between Zf-Zb - no extrapolation is performed.
    //--------------------------------------------------------------

    // (TODO: implement!)
    //void getValueAt( Z, Pixelf&) const;


    //---------------------------------------------
    // Read-only channel value access by ChannelIdx
    //---------------------------------------------

    float   getChannel (size_t segment,
                        ChannelIdx z) const;
    float   getChannel (const DeepSegment& segment,
                        ChannelIdx z) const;


    //------------------------------------------------------------------
    // Flattening (segment compositing)
    //
    // Flatten the DeepSegments down into an output Pixel, determining
    // whether segment overlaps exist.
    // All subpixels are flattened separately and the final output pixel
    // is the weighted accumulation of all subpixel results.
    // If the subpixel mask for all segments are full-coverage then only
    // one flatten operation is performed.
    //
    // 'interpolation' determines the per-segment interpolation
    // behavior - default is INTERP_AUTO.
    //
    // Calls either flattenNoOverlaps() or flattenOverlapping()
    // depending on overlap status.
    //
    // DeepSegments are depth-sorted and composited in front-to-back
    // order. Each segment is composited with the previous using an
    // UNDER or PLUS operation depending on whether the segment is
    // flagged as additive.
    //------------------------------------------------------------------

    void    flatten (const ChannelSet& out_channels,
                     Pixelf& out,
                     InterpolationMode interpolation = INTERP_AUTO);


    //-------------------------------------------------------------------
    // Flattening (segment compositing)
    //
    // Flatten only the DeepSegments that have enabled subpixels matching
    // the input 'spmask' subpixel mask into an output pixel. Segment
    // overlaps are determined and either flattenNoOverlaps() or
    // flattenOverlapping() is called.
    //
    // 'interpolation' determines the per-segment interpolation
    // behavior - default is INTERP_AUTO.
    //
    // DeepSegments are depth-sorted and composited in front-to-back
    // order. Each segment is composited with the previous using an
    // UNDER or PLUS operation depending on whether the segment is
    // flagged as additive.
    //--------------------------------------------------------------------
    void    flattenSubpixels (const ChannelSet& out_channels,
                              Pixelf& out,
                              const SpMask8& spmask,
                              InterpolationMode interpolation = INTERP_AUTO);


    //---------------------------------------------------------------
    // Flattening (segment compositing) with no overlap handling
    //
    // Flatten the DeepSegments for all enabled subpixels in 'spmask'
    // into an output Pixel.
    //
    // DeepSegments are depth-sorted and composited in front-to-back
    // order. Each segment is composited with the previous using an
    // UNDER or PLUS operation depending on whether the segment is
    // flagged as additive.
    //---------------------------------------------------------------
    void    flattenNoOverlaps (const ChannelSet& out_channels,
                               Pixelf& out,
                               const SpMask8& spmask);


    //--------------------------------------------------------------
    // Flattening (segment compositing)
    //
    // 'interpolation' determines the per-segment interpolation
    // behavior - default is INTERP_AUTO.
    //
    // DeepSegments are depth-sorted and composited in front-to-back
    // order. Each segment is composited with the previous using an
    // UNDER or PLUS operation depending on whether the segment is
    // flagged as additive.
    //--------------------------------------------------------------
    void    flattenOverlapping (const ChannelSet& out_channels,
                                Pixelf& out,
                                const SpMask8& spmask,
                                InterpolationMode interpolation = INTERP_AUTO);


    //-------------------------------------------------------
    // Flattening (segment compositing)
    // Use legacy flattening math - has matte support, but no
    // spmask or interpolation-mode support.
    //-------------------------------------------------------
    void    flattenOverlappingLegacy (const ChannelSet& out_channels,
                                      Pixelf& out);


    //---------------------------------------------------------
    // Print info about DeepPixel to output stream
    //---------------------------------------------------------

    void    printInfo (std::ostream&,
                       const char* prefix,
                       int padding=2,
                       bool show_mask=true);
    friend  std::ostream& operator << (std::ostream&,
                                       DeepPixel&);


  protected:

    ChannelSet                  m_channels;         // ChannelSet shared by all segments
    std::vector<DeepSegment>    m_segments;         // List of deep sample segments
    std::vector<Pixelf>         m_pixels;           // List of Pixels referenced by DeepSegment.index
    //
    bool                        m_sorted;           // Have the segments been Z-sorted?
    bool                        m_overlaps;         // Are there any Z overlaps between segments?
    //
    SpMask8                     m_accum_or_mask;    // Subpixel bits that are on for ANY segment
    SpMask8                     m_accum_and_mask;   // Subpixel bits that are on for ALL segments
    DeepFlag                    m_accum_or_flags;   // Deep flags that are on for ANY segment
    DeepFlag                    m_accum_and_flags;  // Deep flags that are on for ALL segments
};



//-----------------
// Inline Functions
//-----------------

inline DeepMetadata::DeepMetadata (SpMask8 _spmask, DeepFlag _flags) :
    spmask(_spmask),
    flags(_flags)
{
    //
}
//--------------------------------------------------------
inline void DeepSegment::setDepths (float _Zf, float _Zb)
{
    Zf = (_Zf <= _Zb)?_Zf:_Zb;
    Zb = _Zb;
    if (Zf < EPSILONf)
    {
        Zf = Zb;
        if (Zf < EPSILONf)
            Zf = Zb = EPSILONf;
    }
}
inline DeepSegment::DeepSegment (float _Zf, float _Zb, int _index, const DeepMetadata& _metadata) :
    index(_index),
    metadata(_metadata)
{
    setDepths(_Zf, _Zb);
}
//
inline DeepFlag DeepSegment::flags () const { return metadata.flags; }
inline bool DeepSegment::isHardSurface () const { return (metadata.flags & DEEP_LINEAR_INTERP_SAMPLE)!=0; }
inline bool DeepSegment::isVolumetric () const { return !isHardSurface(); }
inline bool DeepSegment::isThin () const { return (Zf >= Zb); }
inline bool DeepSegment::isThick () const { return !isThin(); }
inline bool DeepSegment::isMatte () const { return (metadata.flags & DEEP_MATTE_OBJECT_SAMPLE)!=0; }
inline bool DeepSegment::isAdditive () const { return (metadata.flags & DEEP_ADDITIVE_SAMPLE)!=0; }
inline bool DeepSegment::hasPartialSubpixelBinCoverage () const { return (metadata.flags & DEEP_PARTIAL_BIN_COVERAGE)!=0; }
//
inline SpMask8& DeepSegment::spMask () { return metadata.spmask; }
inline const SpMask8& DeepSegment::spMask () const { return metadata.spmask; }
inline bool DeepSegment::zeroCoverage () const { return (metadata.spmask==SpMask8::zeroCoverage); }
inline bool DeepSegment::fullCoverage () const {
    return (metadata.spmask==SpMask8::fullCoverage || metadata.spmask==SpMask8::zeroCoverage);
}
inline bool DeepSegment::hasSpMasks () const { return zeroCoverage(); }
inline bool DeepSegment::maskBitsEnabled (const SpMask8& check_bits) const {
    return (zeroCoverage() || (metadata.spmask & check_bits));
}
inline bool DeepSegment::maskBitEnabled (int bit_index) const {
    return (zeroCoverage() || ((metadata.spmask & (SpMask8(1ull) << bit_index)) != 0));
}
inline float DeepSegment::getCoverage () const {
    if (fullCoverage())
        return 1.0f;
    // Count the on pixels:
    uint32_t on_bits = 0;
    SpMask8 spmask = 1ull;
    for (int sp_bin=0; sp_bin < SpMask8::numBits; ++sp_bin, ++spmask)
        if (metadata.spmask & spmask)
            ++on_bits;
    return (on_bits == 0)?0.0f:float(on_bits)/float(SpMask8::numBits);
}
inline void DeepSegment::applyMask (const ChannelSet& channels, Pixelf& color) {
    const float w = getCoverage();
    foreach_channel(z, channels)
        color[z] *= w;
    metadata.spmask = SpMask8::fullCoverage;
}
//
inline bool DeepSegment::operator < (const DeepSegment& b) const {
    if (Zf < b.Zf)
        return true;
    if (Zf > b.Zf)
        return false;
    return (Zb < b.Zb); // If both Zfronts are equal, check Zback
}
inline void DeepSegment::printFlags (std::ostream& os) const { metadata.printFlags(os); }
//--------------------------------------------------------
inline DeepPixel::DeepPixel (const ChannelSet& channels) :
    m_channels(channels),
    m_sorted(false),
    m_overlaps(false),
    m_accum_or_mask(SpMask8::zeroCoverage),
    m_accum_and_mask(SpMask8::zeroCoverage),
    m_accum_or_flags(DEEP_EMPTY_FLAG),
    m_accum_and_flags(DEEP_EMPTY_FLAG)
{}
inline DeepPixel::DeepPixel (const ChannelIdx channel) :
    m_channels(channel),
    m_sorted(false),
    m_overlaps(false),
    m_accum_or_mask(SpMask8::zeroCoverage),
    m_accum_and_mask(SpMask8::zeroCoverage),
    m_accum_or_flags(DEEP_EMPTY_FLAG),
    m_accum_and_flags(DEEP_EMPTY_FLAG)
{}
inline DeepPixel::DeepPixel (const DeepPixel& b) {
    m_channels        = b.m_channels;
    m_segments        = b.m_segments;
    m_pixels          = b.m_pixels;
    m_sorted          = b.m_sorted;
    m_overlaps        = b.m_overlaps;
    m_accum_or_mask   = b.m_accum_or_mask;
    m_accum_and_mask  = b.m_accum_and_mask;
    m_accum_or_flags  = b.m_accum_or_flags;
    m_accum_and_flags = b.m_accum_and_flags;
}
//
inline const ChannelSet& DeepPixel::channels () const { return m_channels; }
inline void DeepPixel::setChannels (const ChannelSet& channels) {
    const size_t nPixels = m_pixels.size();
    for (size_t i=0; i < nPixels; ++i)
        m_pixels[i].channels = channels;
    m_channels = channels;
}
inline bool DeepPixel::empty () const { return (m_segments.size() == 0); }
inline size_t DeepPixel::size () const { return m_segments.size(); }
inline size_t DeepPixel::capacity () const { return m_segments.capacity(); }
inline void DeepPixel::reserve (size_t n) { m_segments.reserve(n); m_pixels.reserve(n); }
//
inline DeepSegment& DeepPixel::operator [] (size_t segment_index) { return m_segments[segment_index]; }
inline const DeepSegment& DeepPixel::operator [] (size_t segment_index) const { return m_segments[segment_index]; }
inline DeepSegment& DeepPixel::getSegment (size_t segment_index) { return m_segments[segment_index]; }
inline const DeepSegment& DeepPixel::getSegment (size_t segment_index) const { return m_segments[segment_index]; }
//
inline bool DeepPixel::hasSpMasks () { return !allZeroCoverage(); }
//
inline const Pixelf& DeepPixel::getPixel (size_t pixel_index) const { return m_pixels[pixel_index]; }
inline Pixelf& DeepPixel::getPixel (size_t pixel_index) { return m_pixels[pixel_index]; }
inline const Pixelf& DeepPixel::getSegmentPixel (size_t segment_index) const { return m_pixels[m_segments[segment_index].index]; }
inline Pixelf& DeepPixel::getSegmentPixel (size_t segment_index) { return m_pixels[m_segments[segment_index].index]; }
inline const Pixelf& DeepPixel::getSegmentPixel (const DeepSegment& segment) const { return m_pixels[segment.index]; }
inline Pixelf& DeepPixel::getSegmentPixel (const DeepSegment& segment) { return m_pixels[segment.index]; }
inline float DeepPixel::getChannel (size_t segment, ChannelIdx z) const { return m_pixels[m_segments[segment].index][z]; }
inline float DeepPixel::getChannel (const DeepSegment& segment, ChannelIdx z) const { return m_pixels[segment.index][z]; }
//
inline void DeepPixel::setSegmentMask (const SpMask8& spmask, size_t start, size_t end) {
    ++end;
    const size_t nSegments = m_segments.size();
    if (end < start || start >= nSegments)
        return;
    if (end >= nSegments)
        end = nSegments;
    for (size_t i=start; i < end; ++i)
        m_segments[i].metadata.spmask = spmask;
    m_accum_or_mask = m_accum_and_mask = SpMask8::zeroCoverage;
    m_sorted = false; // force it to re-evaluate
}
//
inline void DeepPixel::flattenOverlappingLegacy (const ChannelSet& out_channels, Pixelf& out) {
    flattenOverlapping(out_channels, out, SpMask8::zeroCoverage, INTERP_LOG);
}
//
inline DeepPixel& DeepPixel::operator += (float val) {
    const size_t nSegments = m_segments.size();
    for (size_t i=0; i < nSegments; ++i)
        getSegmentPixel(i) += val;
    return *this;
}
inline DeepPixel& DeepPixel::operator -= (float val) {
    const size_t nSegments = m_segments.size();
    for (size_t i=0; i < nSegments; ++i)
        getSegmentPixel(i) -= val;
    return *this;
}
inline DeepPixel& DeepPixel::operator *= (float val) {
    const size_t nSegments = m_segments.size();
    for (size_t i=0; i < nSegments; ++i)
        getSegmentPixel(i) *= val;
    return *this;
}
inline DeepPixel& DeepPixel::operator /= (float val) {
    const float ival = 1.0f / val;
    const size_t nSegments = m_segments.size();
    for (size_t i=0; i < nSegments; ++i)
        getSegmentPixel(i) *= ival;
    return *this;
}
//--------------------------------------------------------
inline /*static*/
void DeepSegment::interpolateLog (const Pixelf& in, const ChannelSet& channels, float t, Pixelf& out) {
    if (t < EPSILONf) {
        out.erase(channels); // Too thin, no contribution

    } else if (t < 1.0f) {
        const float Ain = in[Dcx::Chan_A];
        if (Ain <= EPSILONf) {
            // If tiny alpha do linear-interpolation:
            foreach_channel(z, channels)
                out[z] = in[z]*t;

        } else if (Ain < 1.0f) {
            // Do log interpolation by converting alpha to density (absorption):
            //   expm1 = 'exp(x) minus 1' & log1p = 'log(1 plus x)'
            //   i.e. x = exp(t * log(1.0 + -x)) - 1.0
            const float Aout = float(-expm1(t * log1p(-Ain)));
            const float w = Aout / Ain;
            foreach_channel(z, channels)
                out[z] = in[z]*w;
            out[Dcx::Chan_A] = Aout;

        } else {
            // Saturated alpha = no interpolation:
            foreach_channel(z, channels)
                out[z] = in[z];
        }

    } else {
        // Entire segment width, max contribution:
        foreach_channel(z, channels)
            out[z] = in[z];
    }
}
inline /*static*/
void DeepSegment::interpolateLin (const Pixelf& in, const ChannelSet& channels, float t, Pixelf& out) {
    if (t < EPSILONf) {
        out.erase(channels); // Too thin, no contribution
    } else if (t < 1.0f) {
        // Do linear interpolation:
        foreach_channel(z, channels)
            out[z] = in[z]*t;
    } else {
        // Entire segment width, max contribution:
        foreach_channel(z, channels)
            out[z] = in[z];
    }
}
inline
void DeepSegment::interpolate (Pixelf& in, const ChannelSet& channels, float t, Pixelf& out) const {
    if (isHardSurface())
        interpolateLin(in, channels, t, out);
    else
        interpolateLog(in, channels, t, out);
}

OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DEEPPIXEL_H
