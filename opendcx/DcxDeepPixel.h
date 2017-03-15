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

//=============================================================================
//
//  struct  DeepMetadata
//
//  class   DeepSegment
//  struct  DeepSegment::Edge
//
//  class   DeepPixel
//  enum    DeepPixel::InterpolationMode
//
//  typedef SegmentEdgeList;
//  typedef SegmentEdgeSet
//
//=============================================================================

#include "DcxChannelSet.h"
#include "DcxChannelDefs.h"
#include "DcxPixel.h"
#include "DcxSpMask.h"
#include "DcxDeepFlags.h"

#include <iostream>
#include <vector>
#include <math.h> // for log1p, expm1


//-------------------------
//!rst:cpp:begin::
//DeepPixel
//=========
//-------------------------


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//-------------------------
//!rst:left-align::
//.. _deepmetadata_class:
//
//DeepMetadata
//************
//-------------------------

//========================
//
//  struct DeepMetadata
// 
//========================
//-------------------------------------------------------------------------------
//
//  Stores extra information about a DeepSegment.
//  For the moment it only stores the subpixel mask and the flags, but this
//  could be made to hold arbitrary attributes.
//
//  These values are packed & unpacked via float & half channels for file IO.
//
//  TODO: support arbitrary attributes?
//
//-------------------------------------------------------------------------------

struct DCX_EXPORT DeepMetadata
{


    SpMask8     spmask;         // Subpixel 8x8 bitmask array (A-buffer)
    DeepFlags   flags;          // Flags - interpolation-type, matte-mode, etc.


    //-----------------------------------------
    // Default constructor leaves junk in vars.
    //-----------------------------------------

    DeepMetadata ();


    //------------
    // Constructor
    //------------

    DeepMetadata (const SpMask8& _mask,
                  const DeepFlags& _flags);


    //-------------------------------------------------------
    // Is Z-depth of segment zero (thin) or non-zero (thick)?
    //-------------------------------------------------------

    bool    isThin () const;
    bool    isThick () const;
    float   thickness () const;


    //-----------------------------------------------------------------
    // Does the segment represent a solid (hard) or volumetric surface?
    // If hard-surface - use linear interpolation between Zf/Zb.
    // If volumetric - use log interpolation between Zf/Zb.
    //-----------------------------------------------------------------

    bool    isHardSurface () const;
    bool    isVolumetric () const;


    //-------------------------------------------------------------------
    // Should the segment cutout (act as a matte) the segments behind it?
    //-------------------------------------------------------------------

    bool    isMatte () const;


    //-------------------------------------------------------------------
    // Sample should be added to surrounding samples rather than under-ed
    // This is used primarily for partial subpixel-coverage.
    //-------------------------------------------------------------------

    bool    isAdditive () const;


    //-----------------------------------------------------------------------------
    // Does the sample have partial subpixel coverage baked into color, alpha, etc?
    // If so the sample is handled as additive when composited.
    // This normally indicates filtering or resampling of subpixel masks has been
    // applied.
    //-----------------------------------------------------------------------------

    uint32_t    surfaceFlags () const;
    uint32_t    partialCoverageBits () const;

    bool        hasPartialSpCoverage () const;
    bool        hasFullSpCoverage () const;

    uint32_t    getSpCoverageCount () const;
    float       getSpCoverageWeight () const;

    void        setSpCoverageCount (uint32_t count);
    void        setSpCoverageWeight (float weight);
    void        clearSpCoverageCount ();


    //--------------------------------
    // Print the list of enabled flags
    //--------------------------------

    void    printFlags (std::ostream&) const;
    friend  std::ostream& operator << (std::ostream&,
                                       const DeepMetadata&);

};


//-------------------------
//!rst:left-align::
//.. _deepsegment_class:
//
//DeepSegment
//***********
//-------------------------

//=====================
//
//  class DeepSegment
// 
//=====================
//-------------------------------------------------------------------------------------
//
//  A single deep sample describing a linear segment of Z-space where Zf <= Zb.
//
//  The color channel data describes the values at Zb, so finding values between
//  Zf and Zb requires linear or log interpolation depending on the
//  interpolation-type flag in the metadata.
//
//  Note that this class does not store the actual channel data so that the adding,
//  deleting, sorting, etc of DeepSegments is lightweight and fast.
//
//-------------------------------------------------------------------------------------

class DCX_EXPORT DeepSegment
{
  public:

    //----------------------------------------------------------
    //  struct DeepSegment::Edge
    //    Two of these are created for each DeepSegment to track
    //    the active segments during merging routines.
    //----------------------------------------------------------

    struct Edge
    {
        enum Type
        {
            THIN  = -1,     // Segment has no depth (Zf == Zb, depth = Zf)
            FRONT =  0,     // Front edge of segment (depth = Zf)
            BACK  =  1      // Back edge of segment (depth = Zb)
        };

        float       depth;
        uint32_t    segment;
        Type        type;

        Edge(float _depth, uint32_t _segment, Type _type);

        bool operator < (const Edge& b) const;
    };


  public:

    float           Zf, Zb;         // Z-front / Z-back depth positions
    int             index;          // Index into an array of channel data storage. -1 == non-assignment!
    DeepMetadata    metadata;       // Flags - interpolation-type, matte-mode, etc.


  public:
    //-----------------------------------------
    // Default constructor leaves junk in vars.
    //-----------------------------------------
    DeepSegment ();


    //---------------------------------------
    // Constructor
    // If Zf > Zb, Zf is clamped to Zb.
    //---------------------------------------

    DeepSegment (float _Zf, float _Zb,
                 int _index = -1,
                 const DeepMetadata& _metadata = DeepMetadata(SpMask8::fullCoverage,
                                                              DeepFlags::ALL_BITS_OFF));


    //--------------------------------------------
    // Set both Zf & Zb - checks for bad Z's and
    // tries to find reasonable solution...
    //--------------------------------------------

    void    setDepths (float _Zf, float _Zb);

    void    transformDepths (float translate,
                             float scale,
                             bool reverse=false); // translate then scale, or scale then translate if reverse=true

    //-------------------------
    // Used by the sort routine
    //-------------------------

    bool operator < (const DeepSegment&) const;


    //-------------------------
    // DeepFlags metadata access
    //-------------------------

    const DeepFlags&    flags () const;
    uint32_t            surfaceFlags () const;


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
    float   thickness () const;

    //-------------------------------------------------------
    // Should the segment cutout (act as a matte) the segments behind it?
    //-------------------------------------------------------

    bool    isMatte () const;


    //-------------------------------------------------------------------
    // Sample should be added to surrounding samples rather than under-ed
    //-------------------------------------------------------------------

    bool    isAdditive () const;


    //-----------------------------------------------------------------------------
    // Does the sample have partial subpixel coverage baked-in to color, alpha, etc
    // values? If so the sample is handled as additive when composited.
    // This normally indicates filtering or resampling of subpixel masks has been
    // applied.
    //-----------------------------------------------------------------------------

    bool        hasPartialSpCoverage () const;
    bool        hasFullSpCoverage () const;

    uint32_t    getSpCoverageCount () const;
    float       getSpCoverageWeight () const;

    void        setSpCoverageCount (uint32_t count);
    void        setSpCoverageWeight (float weight);
    void        clearSpCoverageCount ();


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
    // Note that all-bits-off is considered all-bits-on for these tests.
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

    void    interpolate (const Pixelf& in,
                         float t,
                         const ChannelSet& do_channels,
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
    //------------------------------------------------------------

    static void interpolateLog (const Pixelf& in,
                                float t,
                                const ChannelSet& do_channels,
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
    //-----------------------------------------------------------

    static void     interpolateLin (const Pixelf& in,
                                    float t,
                                    const ChannelSet& do_channels,
                                    Pixelf& out);


    //--------------------------------------------------------------
    // Log-merge two samples.
    // Volumetric log merging math from Florian's deep doc adapted
    // for channel loops.
    // 'A' is the first merge source
    // 'B' is the second merge source and the output (it's modified)
    //--------------------------------------------------------------

    static void mergeLog (const Pixelf& A,
                          const ChannelSet& channels,
                          Pixelf& B);


    //-----------------------------------------------------------
    // Print info about DeepSegment to output stream
    //-----------------------------------------------------------

    void    printInfo (std::ostream&,
                       bool show_mask=true) const;
    void    printFlags (std::ostream&) const;
    friend  std::ostream& operator << (std::ostream&,
                                       const DeepSegment&);

};


typedef std::vector<DeepSegment::Edge> SegmentEdgeList;
typedef std::set<uint32_t> SegmentEdgeSet;



//-------------------------
//!rst:left-align::
//.. _deeppixel_class:
//
//DeepPixel
//*********
//-------------------------

//=====================
//
//  class DeepPixel
// 
//=====================
//----------------------------------------------------------------------------------------
//
//  Contains a series of DeepSegments and their associated Pixels (containing float
//  channel data,) which combined together comprise a series of deep samples.
//
//  Supports metadata storage for each deep sample and offers methods to aid the
//  manipulation of samples within the deep pixel, including compositing (flattening,)
//  and sorting.
//  Because a :ref:`deepsegment_class` is lightweight the list can be rearranged and
//  sorted very quickly.  The list of large Pixel channel data structures is kept
//  static.
//
//  ::
//
//      TODO: investigate cost of using varying-sized Pixel channel array
//      TODO: extend flatten methods to accept near/far depth range to flatten within
//
//----------------------------------------------------------------------------------------

class DCX_EXPORT DeepPixel
{
  public:

    //------------------------------------------------------
    // Segment interpolation modes for flattening operations
    //------------------------------------------------------

    enum InterpolationMode
    {
        INTERP_OFF,         // Disable interpolation
        INTERP_AUTO,        // Determine interpolation from per-sample metadata (DeepFlags)
        INTERP_LOG,         // Use log interpolation for all samples
        INTERP_LIN,         // Use linear interpolation for all samples

        NUM_INTERPOLATIONMODES
    };

    static const char* interpolationModeString (InterpolationMode mode);


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


    //---------------------------------------------------------
    // Get/set the xy location
    //      Currently used for debugging
    //---------------------------------------------------------

    int     x () const;
    int     y () const;
    void    getXY (int& x, int& y);

    void    setXY (int x, int y);


    //---------------------------------------------------------
    // Transform the Zf & Zb coords for all segments.
    //---------------------------------------------------------

    void    transformDepths (float translate,
                             float scale,
                             bool reverse=false); // translate then scale, or scale then translate if reverse=true


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
    void    invalidateSort ();

    // Return the index of the DeepSegment nearest to Z and inside the distance of Z +- maxDistance, or -1 if nothing found.
    // (TODO: finish implementing this!)
    //int     nearestSegment (double Z, double maxDistance=0.0);

    // Check for overlaps between samples and return true if so.
    bool    hasOverlaps (const SpMask8& spmask=SpMask8::fullCoverage, bool force=false);

    // Returns true if at least one segment has spmasks (same as !allZeroCoverage())
    bool    hasSpMasks ();

    // Returns true if all segment spmasks are zero-coverage and not hard-surface/matte/additive tagged.
    bool    isLegacyDeepPixel ();

    // Returns true if all segment spmasks are zero-coverage - this may indicate a legacy deep pixel.
    bool    allZeroCoverage ();

    // Returns true if all segment spmasks are full-coverage, zero-coverage or a mix of full OR zero coverage.
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
    size_t  append (const DeepSegment& segment);
    // Add a DeepSegment to the end of the list.
    size_t  append (const DeepSegment& segment, const Pixelf& pixel);
    // Copy one segment from the second DeepPixel.
    size_t  append (const DeepPixel& b, size_t segment_index);
    // Combine all the segments of two DeepPixels.
    void    append (const DeepPixel& b);

    //
    // Append or combine a segment with the existing segments.
    //
    void    appendOrCombineSegment (const DeepSegment& segment,
                                    const Pixelf& pixel,
                                    float depth_threshold=0.001f,
                                    float color_threshold=0.001f);

    //
    // Append or combine a segment with a subset list of existing segments.
    //
    void    appendOrCombineSegment (const std::vector<int>& segment_indices,
                                    const DeepSegment& segment,
                                    const Pixelf& pixel,
                                    float color_threshold=0.001f);

    //
    // Remove a DeepSegment from the segment list, deleting its referenced Pixel.
    // Note that this method will possibly reorder some of the Pixel indices in the
    // DeepSegments, so a previously referenced Pixel index may become invalid and
    // need to be reaquired from its DeepSegment.
    //
    void    removeSegment (size_t segment_index);


    //----------------------------------------------
    // Accumulated spmask and flags for all segments
    //----------------------------------------------

    SpMask8     getAccumOrMask () const;
    SpMask8     getAccumAndMask () const;

    DeepFlags   getAccumOrFlags () const;
    DeepFlags   getAccumAndFlags () const;


    //------------------------------------------------------------------
    // Segment Searches
    //------------------------------------------------------------------

    // Get the list of segment indices that match the Zf/Zb values.
    void    findNearestMatches (const DeepSegment& segment,
                                float depth_threshold,
                                std::vector<int>& matched_segments_list);

    void    findNearestMatches (float Zf, float Zb,
                                float depth_threshold,
                                std::vector<int>& matched_segments_list);

    // Find the best DeepSegment that matches the given segment & pixel.
    // Returns the DeepSegment index or -1 if no match.

    int     findNearestMatch (const DeepSegment& match_segment,
                              uint32_t start_index=0,
                              float depth_threshold=EPSILONf);

    int     findNearestMatch (float Zf, float Zb,
                              const SpMask8& match_spmask=SpMask8::fullCoverage,
                              DeepFlags match_flags=DeepFlags::ALL_BITS_OFF,
                              uint32_t start_index=0,
                              float depth_threshold=EPSILONf);

    int     findBestMatch (const DeepSegment& match_segment,
                           const Pixelf& match_color,
                           const ChannelSet& compare_channels,
                           const SpMask8& match_spmask=SpMask8::fullCoverage,
                           DeepFlags match_flags=DeepFlags::ALL_BITS_OFF,
                           uint32_t start_index=0,
                           float color_threshold=EPSILONf,
                           float depth_threshold=EPSILONf);


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

    //------------------------------------------------------------------------
    // These Pixel read methods also copy metadata values from the DeepSegment
    // into the output Pixel's predefined channels: Chan_ZFront, Chan_ZBack,
    // Chan_SpBits1, Chan_SpBits2, Chan_DeepFlags and Chan_SpCoverage.
    //------------------------------------------------------------------------

    void    getSegmentPixelWithMetadata (size_t segment_index,
                                         Pixelf& out) const;
    void    getSegmentPixelWithMetadata (size_t segment_index,
                                         const ChannelSet& get_channels, // restrict copied channels to this
                                         Pixelf& out) const;
    void    getSegmentPixelWithMetadata (const DeepSegment& segment,
                                         Pixelf& out) const;
    void    getSegmentPixelWithMetadata (const DeepSegment& segment,
                                         const ChannelSet& get_channels, // restrict copied channels to this
                                         Pixelf& out) const;


    //--------------------------------------------------------------
    // Get a Pixel sampled at depth Z, possibly using interpolation.
    // Z is clamped between Zf-Zb - no extrapolation is performed.
    // (TODO: implement!)
    //--------------------------------------------------------------

    //void    getValueAt (Z, Pixelf&) const;


    //---------------------------------------------
    // Read-only channel value access by ChannelIdx
    //---------------------------------------------

    float   getChannel (size_t segment,
                        ChannelIdx z) const;
    float   getChannel (const DeepSegment& segment,
                        ChannelIdx z) const;


    //--------------------------------------------------------------
    // Build / update SegmentEdgeSet
    // Only segments with enabled sp bits in 'spmask' will be
    // added to the edge set.
    // Returns the number of built edges.
    //--------------------------------------------------------------

    size_t  buildSegmentEdges (const SpMask8& spmask,
                               SegmentEdgeList& segment_set);


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


    //-------------------------------------------------------------------------
    // Segment compositing
    //
    // Composite segment 'A' underneath accumulation pixel 'B'. Normally
    // used for front-to-back flattening of non-overlapping segments.
    // The ChannelSet should contain only the color channels to
    // composite - accumulation alpha/viz/depth and cutout alpha
    // are handled explicitly.
    //
    // Before calling this method on the first segment, initialize
    // the color and accumulation channels in B like so:
    // ::
    //
    //     B.erase(comp_color_channels);
    //     B[Chan_A         ] = 0.0f;
    //     B[Chan_ACutout   ] = 0.0f;
    //     B[Chan_Visibility] = 1.0f;
    //     B[Chan_SpCoverage] = 0.0f;
    //     B[Chan_ZFront    ] =  INFINITYf;
    //     B[Chan_ZBack     ] = -INFINITYf;
    //
    //
    // After the method runs they will contain:
    // ::
    //
    //     B[Chan_A         ] = Accumulated alpha
    //     B[Chan_ACutout   ] = Accumulated alpha with cutouts (holes) applied
    //     B[Chan_Visibility] = Accumulated visibility (1 - B[Chan_A]),
    //                          updated when accumulated subpixel
    //                          coverage weight is >= 1.0.
    //     B[Chan_SpCoverage] = Accumulated subpixel coverage weight
    //     B[Chan_ZFront    ] = Minimum Chan_ZFront depth
    //     B[Chan_ZBack     ] = Maximum Chan_ZBack depth
    //
    //--------------------------------------------------------------------------

    void    compositeSegmentUnder (Pixelf& B,
                                   const DeepSegment& segment,
                                   const ChannelSet& color_channels);

    void    compositeSegmentUnder (Pixelf& B,
                                   const DeepSegment& segment,
                                   const Pixelf& A,
                                   const ChannelSet& color_channels);

    void    compositeSegmentUnder (Pixelf& B,
                                   float Zf, float Zb,
                                   DeepFlags flags,
                                   const Pixelf& A,
                                   const ChannelSet& color_channels);

    void    compositeSegmentUnder (Pixelf& B,
                                   float Zf, float Zb,
                                   DeepFlags flags,
                                   const Pixelf& A,
                                   float Aalpha,       /* overrides A[Chan_A]          */
                                   float Acutout,      /* overrides A[Chan_ACutout]    */
                                   float Aspcoverage,  /* overrides A[Chan_SpCoverage] */
                                   const ChannelSet& color_channels);


    //---------------------------------------------------------------
    // Legacy segment merging
    //
    // Segment 'subsegment' contains the Zf-Zb depth range to merge
    // which must fall within the min/max depth range of the segment
    // set - no checks are performed to verify this!
    // 
    //---------------------------------------------------------------

    void    mergeSectionLegacy (const SegmentEdgeSet& segment_set,
                                double Zf,
                                double Zb,
                                const ChannelSet& color_channels,
                                Pixelf& out,
                                DeepMetadata& merged_metadata);


    //-------------------------------------------------------------
    // Extract the color from a segment subsection.
    // 
    // The output pixel is a copy of the segment pixel with changes
    // to only the interpolated channels.
    //-------------------------------------------------------------

    void    extractSubSectionLog (const DeepSegment& segment,
                                  double Zf,
                                  double Zb,
                                  const ChannelSet& interpolate_channels,
                                  Pixelf& out);
    void    extractSubSectionLog (const DeepSegment& segment,
                                  const DeepSegment& subsegment,
                                  const ChannelSet& interpolate_channels,
                                  Pixelf& out);

    void    extractSubSectionLin (const DeepSegment& segment,
                                  double Zf,
                                  double Zb,
                                  const ChannelSet& interpolate_channels,
                                  Pixelf& out);
    void    extractSubSectionLin (const DeepSegment& segment,
                                  const DeepSegment& subsegment,
                                  const ChannelSet& interpolate_channels,
                                  Pixelf& out);

    void    extractSubSection (const DeepSegment& segment,
                               const DeepSegment& subsegment,
                               const ChannelSet& interpolate_channels,
                               Pixelf& out);
    void    extractSubSection (const DeepSegment& segment,
                               double Zf,
                               double Zb,
                               const ChannelSet& interpolate_channels,
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
    int                         m_x, m_y;           // Pixel's xy location
    bool                        m_sorted;           // Have the segments been Z-sorted?
    bool                        m_overlaps;         // Are there any Z overlaps between segments?
    //
    SpMask8                     m_accum_or_mask;    // Subpixel bits that are on for ANY segment
    SpMask8                     m_accum_and_mask;   // Subpixel bits that are on for ALL segments
    DeepFlags                   m_accum_or_flags;   // Deep flags that are on for ANY segment
    DeepFlags                   m_accum_and_flags;  // Deep flags that are on for ALL segments
};



//--------------
//!rst:cpp:end::
//--------------

//-----------------
// Inline Functions
//-----------------

inline DeepMetadata::DeepMetadata () {}
inline DeepMetadata::DeepMetadata (const SpMask8& _spmask, const DeepFlags& _flags) :
    spmask(_spmask),
    flags(_flags)
{
    //
}
inline bool DeepMetadata::isHardSurface () const { return flags.isHardSurface(); }
inline bool DeepMetadata::isVolumetric () const { return flags.isVolumetric(); }
inline bool DeepMetadata::isMatte () const { return flags.isMatte(); }
inline bool DeepMetadata::isAdditive () const { return flags.isAdditive(); }
inline bool DeepMetadata::hasFullSpCoverage () const { return flags.hasFullSpCoverage(); }
inline bool DeepMetadata::hasPartialSpCoverage () const { return flags.hasPartialSpCoverage(); }
inline float DeepMetadata::getSpCoverageWeight () const { return flags.getSpCoverageWeight(); }
inline uint32_t DeepMetadata::getSpCoverageCount () const { return flags.getSpCoverageCount(); }
inline void DeepMetadata::setSpCoverageCount (uint32_t count) { flags.setSpCoverageCount(count); }
inline void DeepMetadata::setSpCoverageWeight (float weight) { flags.setSpCoverageWeight(weight); }
inline void DeepMetadata::clearSpCoverageCount () { flags.clearSpCoverageCount(); }
inline void DeepMetadata::printFlags (std::ostream& os) const { flags.print(os); }
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
inline DeepSegment::DeepSegment () {}
inline DeepSegment::DeepSegment (float _Zf, float _Zb, int _index, const DeepMetadata& _metadata) :
    index(_index),
    metadata(_metadata)
{
    setDepths(_Zf, _Zb);
}
//
inline void DeepSegment::transformDepths (float translate, float scale, bool reverse) {
    if (!reverse)
    {
        Zf = (Zf + translate)*scale;
        Zb = (Zb + translate)*scale;
    }
    else
    {
        Zf = Zf*scale + translate;
        Zb = Zb*scale + translate;
    }
}
//
inline const DeepFlags& DeepSegment::flags () const { return metadata.flags; }
inline uint32_t DeepSegment::surfaceFlags () const { return metadata.flags.surfaceFlags(); }
inline bool DeepSegment::isHardSurface () const { return metadata.isHardSurface(); }
inline bool DeepSegment::isVolumetric () const { return metadata.isVolumetric(); }
inline bool DeepSegment::isThin () const { return (Zb <= Zf); }
inline bool DeepSegment::isThick () const { return !isThin(); }
inline float DeepSegment::thickness () const { return (Zb - Zf); }
inline bool DeepSegment::isMatte () const { return metadata.isMatte(); }
inline bool DeepSegment::isAdditive () const { return metadata.isAdditive(); }
inline bool DeepSegment::hasPartialSpCoverage () const { return metadata.hasPartialSpCoverage(); }
inline bool DeepSegment::hasFullSpCoverage () const { return metadata.hasFullSpCoverage(); }
inline float DeepSegment::getSpCoverageWeight () const { return metadata.getSpCoverageWeight(); }
inline uint32_t DeepSegment::getSpCoverageCount () const { return metadata.getSpCoverageCount(); }
inline void DeepSegment::setSpCoverageCount (uint32_t count) { metadata.setSpCoverageCount(count); }
inline void DeepSegment::setSpCoverageWeight (float weight) { metadata.setSpCoverageWeight(weight); }
inline void DeepSegment::clearSpCoverageCount () { metadata.clearSpCoverageCount(); }
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
    if      (Zf < b.Zf) return true;
    else if (Zf > b.Zf) return false;
    // If both Zfronts are equal check partial subpixel-coverage first, then Zbacks
    else if (getSpCoverageCount() < b.getSpCoverageCount()) return true;
    return (Zb < b.Zb);
}
inline void DeepSegment::printFlags (std::ostream& os) const { metadata.printFlags(os); }
//--------------------------------------------------------
inline DeepSegment::Edge::Edge(float _depth, uint32_t _segment, DeepSegment::Edge::Type _type) :
    depth(_depth),
    segment(_segment),
    type(_type)
{
    //
}
inline bool DeepSegment::Edge::operator < (const DeepSegment::Edge& b) const
{
    if      (depth   < b.depth  ) return true;
    else if (depth   > b.depth  ) return false;
    else if (segment < b.segment) return true;
    else if (segment > b.segment) return false;
    return (type < b.type);
}
//--------------------------------------------------------
inline DeepPixel::DeepPixel (const ChannelSet& channels) :
    m_channels(channels),
    m_x(0), m_y(0),
    m_sorted(false),
    m_overlaps(false),
    m_accum_or_mask(SpMask8::zeroCoverage),
    m_accum_and_mask(SpMask8::zeroCoverage),
    m_accum_or_flags(DeepFlags::ALL_BITS_OFF),
    m_accum_and_flags(DeepFlags::ALL_BITS_OFF)
{}
inline DeepPixel::DeepPixel (const ChannelIdx channel) :
    m_channels(channel),
    m_x(0), m_y(0),
    m_sorted(false),
    m_overlaps(false),
    m_accum_or_mask(SpMask8::zeroCoverage),
    m_accum_and_mask(SpMask8::zeroCoverage),
    m_accum_or_flags(DeepFlags::ALL_BITS_OFF),
    m_accum_and_flags(DeepFlags::ALL_BITS_OFF)
{}
inline DeepPixel::DeepPixel (const DeepPixel& b) {
    m_channels        = b.m_channels;
    m_segments        = b.m_segments;
    m_pixels          = b.m_pixels;
    m_x               = b.m_x;
    m_y               = b.m_y;
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
//
inline int DeepPixel::x () const { return m_x; }
inline int DeepPixel::y () const { return m_y; }
inline void DeepPixel::getXY (int& x, int& y) { x = m_x; y = m_y; }
inline void DeepPixel::setXY (int x, int y) { m_x = x; m_y = y; }
//
inline void DeepPixel::transformDepths (float translate, float scale, bool reverse) {
    const size_t nSegments = m_segments.size();
    for (size_t i=0; i < nSegments; ++i)
        m_segments[i].transformDepths(translate, scale, reverse);
}
//
inline int DeepPixel::findNearestMatch (const DeepSegment& match_segment, uint32_t start_index, float depth_threshold) {
    return findNearestMatch(match_segment.Zf, match_segment.Zb, SpMask8::fullCoverage, match_segment.flags(), start_index, depth_threshold);
}
//
inline SpMask8 DeepPixel::getAccumOrMask () const { return m_accum_or_mask; }
inline SpMask8 DeepPixel::getAccumAndMask () const { return m_accum_and_mask; }
inline DeepFlags DeepPixel::getAccumOrFlags () const { return m_accum_or_flags; }
inline DeepFlags DeepPixel::getAccumAndFlags () const { return m_accum_and_flags; }
//
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
// Copy Pixel channels in ChannelSet 'get_channels' and copy/extract metadata
// values from the DeepSegment into the output Pixel's predefined channels.
inline void DeepPixel::getSegmentPixelWithMetadata (const DeepSegment& segment, const ChannelSet& get_channels, Pixelf& out) const
{
    out.copy(m_pixels[segment.index], get_channels);
    out.channels = get_channels;
    out[OPENDCX_INTERNAL_NAMESPACE::Chan_ZFront] = segment.Zf;
    out[OPENDCX_INTERNAL_NAMESPACE::Chan_ZBack ] = segment.Zb;
    segment.metadata.spmask.toFloat(out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpBits1],
                                    out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpBits2]);
    out[OPENDCX_INTERNAL_NAMESPACE::Chan_DeepFlags] = segment.metadata.flags.toFloat();
    if (segment.hasPartialSpCoverage())
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = segment.getSpCoverageWeight();
    else
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
    if (segment.isMatte())
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout] = 0.0f;
    else
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout] = out[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
}
inline void DeepPixel::getSegmentPixelWithMetadata (size_t segment_index, Pixelf& out) const {
    const DeepSegment& segment = m_segments[segment_index];
    getSegmentPixelWithMetadata(segment, m_pixels[segment.index].channels, out);
}
inline void DeepPixel::getSegmentPixelWithMetadata (size_t segment_index, const ChannelSet& get_channels, Pixelf& out) const {
    getSegmentPixelWithMetadata(m_segments[segment_index].index, get_channels, out);
}
inline void DeepPixel::getSegmentPixelWithMetadata (const DeepSegment& segment, Pixelf& out) const {
    getSegmentPixelWithMetadata(segment, m_pixels[segment.index].channels, out);
}
inline float DeepPixel::getChannel (size_t segment, ChannelIdx z) const { return m_pixels[m_segments[segment].index][z]; }
inline float DeepPixel::getChannel (const DeepSegment& segment, ChannelIdx z) const { return m_pixels[segment.index][z]; }
//
inline void DeepPixel::setSegmentMask (const SpMask8& spmask, size_t start, size_t end)
{
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
void DeepSegment::interpolateLog (const Pixelf& in, float t, const ChannelSet& channels, Pixelf& out)
{
    if (t < EPSILONf)
    {
        out.erase(channels); // Too thin, no contribution
    }
    else if (t < 1.0f)
    {
        const float Ain = in[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
        if (Ain <= EPSILONf)
        {
            // If tiny alpha do linear-interpolation:
            foreach_channel(z, channels)
                out[z] = in[z]*t;
        }
        else if (Ain < 1.0f)
        {
            // Do log interpolation by converting alpha to density (absorption):
            //   expm1 = 'exp(x) minus 1' & log1p = 'log(1 plus x)'
            //   i.e. x = exp(t * log(1.0 + -x)) - 1.0
            const float Aout = float(-expm1(t * log1p(-Ain)));
            const float w = Aout / Ain;
            foreach_channel(z, channels)
                out[z] = in[z]*w;
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = Aout;
        }
        else
            out.copy(in, channels); // Saturated alpha = no interpolation
    }
    else
        out.copy(in, channels); // Entire segment width, max contribution
}
inline /*static*/
void DeepSegment::interpolateLin (const Pixelf& in, float t, const ChannelSet& channels, Pixelf& out)
{
    if (t < EPSILONf)
    {
        out.erase(channels); // Too thin, no contribution
    }
    else if (t < 1.0f)
    {
        // Do linear interpolation:
        foreach_channel(z, channels)
            out[z] = in[z]*t;
    }
    else
        out.copy(in, channels); // Entire segment width, max contribution
}
inline
void DeepSegment::interpolate (const Pixelf& in, float t, const ChannelSet& channels, Pixelf& out) const
{
    if (isHardSurface())
        interpolateLin(in, t, channels, out);
    else
        interpolateLog(in, t, channels, out);
}
//--------------------------------------------------------
inline
void DeepPixel::extractSubSectionLog (const DeepSegment& segment,
                                      double Zf, double Zb,
                                      const ChannelSet& interpolate_channels,
                                      Pixelf& out)
{
    getSegmentPixelWithMetadata(segment, interpolate_channels, out);
    // Position of Zf/Zb within segment doesn't matter, we just use the thickness:
    segment.interpolateLog(out, ((Zb - Zf) / segment.thickness()), interpolate_channels, out);

    if (segment.isMatte())
    {
        // Matte object - blacken the color channels & cutouts but not opacities:
        const float Aa = out[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
        const float Asp = out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage];
        out.erase(interpolate_channels);
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = Aa;
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = Asp;
    }
    else
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout] = out[OPENDCX_INTERNAL_NAMESPACE::Chan_A];

    if (!segment.hasPartialSpCoverage())
        out[Chan_SpCoverage] = 1.0f; // non-partial samples are always 1.0 coverage
}
inline
void DeepPixel::extractSubSectionLog (const DeepSegment& segment,
                                      const DeepSegment& subsegment,
                                      const ChannelSet& interpolate_channels,
                                      Pixelf& out)
{
    extractSubSectionLog(segment, subsegment.Zf, subsegment.Zb, interpolate_channels, out);
}
//------------------------------
inline
void DeepPixel::extractSubSectionLin (const DeepSegment& segment,
                                      double Zf, double Zb,
                                      const ChannelSet& interpolate_channels,
                                      Pixelf& out)
{
    getSegmentPixelWithMetadata(segment, interpolate_channels, out);
    // Split segment twice to get the correct weight at subsegment.Zb.
    // Interpolate color at Zf & Zb, then un-under Zb color from Zf color:
    const double segmentZf = double(segment.Zf);
    const double segment_thickness = 1.0 / (double(segment.Zb) - segmentZf);
    const double tf = (Zf - segmentZf)*segment_thickness;
    const double tb = (Zb - segmentZf)*segment_thickness;
    const double Aa = out[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
    const double Asp = out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage];
    const double Ba = Aa*tf;
    if (segment.isMatte())
    {
        // Matte object - blacken the color channels & cutouts but
        // UN-UNDER opacities explicitly:
        out.erase(interpolate_channels);
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        if (segment.isAdditive() || Ba <= 0.0)
        {
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = float(Aa*tb - Aa*tf);
            if (segment.hasPartialSpCoverage())
                out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = float(Asp*tb - Asp*tf);
            else
                out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        }
        else if (Ba < 1.0)
        {
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = float((Aa*tb - Aa*tf) / (1.0 - Ba));
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        }
        else
        {
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = 0.0f;
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        }
    }
    else
    {
        // UN-UNDER all channels - and opacities explicitly:
        if (segment.isAdditive() || Ba <= 0.0)
        {
            foreach_channel(z, interpolate_channels)
                out[z] = float(out[z]*tb - out[z]*tf);
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = float(Aa*tb - Aa*tf);
            if (segment.hasPartialSpCoverage())
                out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = float(Asp*tb - Asp*tf);
            else
                out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        }
        else if (Ba < 1.0)
        {
            const double iBa = 1.0 / (1.0 - Ba);
            foreach_channel(z, interpolate_channels)
                out[z] = float((out[z]*tb - out[z]*tf)*iBa);
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = float((Aa*tb - Aa*tf)*iBa);
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        }
        else
        {
            foreach_channel(z, interpolate_channels)
                out[z] = 0.0f;
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = 0.0f;
            out[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
        }
        out[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout] = out[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
    }
}
inline
void DeepPixel::extractSubSectionLin (const DeepSegment& segment,
                                      const DeepSegment& subsegment,
                                      const ChannelSet& interpolate_channels,
                                      Pixelf& out)
{
    extractSubSectionLin(segment, subsegment.Zf, subsegment.Zb, interpolate_channels, out);
}
//----------------------------
inline
void DeepPixel::extractSubSection (const DeepSegment& segment,
                                   double Zf, double Zb,
                                   const ChannelSet& interpolate_channels,
                                   Pixelf& out)
{
    if (segment.isHardSurface())
        extractSubSectionLin(segment, Zf, Zb, interpolate_channels, out);
    else
        extractSubSectionLog(segment, Zf, Zb, interpolate_channels, out);
}
inline
void DeepPixel::extractSubSection (const DeepSegment& segment,
                                   const DeepSegment& subsegment,
                                   const ChannelSet& interpolate_channels,
                                   Pixelf& out)
{
    extractSubSection(segment, subsegment.Zf, subsegment.Zb, interpolate_channels, out);
}
//--------------------------------------------------------
inline /*static*/
void DeepSegment::mergeLog (const Pixelf& A, const ChannelSet& channels, Pixelf& B)
{
    // Volumetric log merging math from Florian's deep doc adapted for channel loops
    const float Balpha = B[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
    const float Aalpha = A[OPENDCX_INTERNAL_NAMESPACE::Chan_A];
    const float mergedAlpha = (Balpha + Aalpha) - (Balpha * Aalpha);
    if (Balpha >= 1.0f && Aalpha >= 1.0f)
    {
        // Max opacity, average them:
        foreach_channel(z, channels)
            B[z] = (B[z] + A[z]) / 2.0f;
    }
    else if (Balpha >= 1.0f)
    {
        // No merge, leave as-is
    }
    else if (Aalpha >= 1.0f)
    {
        B.copy(A, channels); // No merge, copy sample
    }
    else
    {
        // Log merge:
        foreach_channel(z, channels)
        {
            if (z == OPENDCX_INTERNAL_NAMESPACE::Chan_A)
                continue;
            static const float MAXF = std::numeric_limits<float>::max();
            const float u1 = float(-log1p(-Balpha));
            const float v1 = (u1 < Balpha*MAXF)?u1/Balpha:1.0f;
            const float u2 = float(-log1p(-Aalpha));
            const float v2 = (u2 < Aalpha*MAXF)?u2/Aalpha:1.0f;
            const float u = u1 + u2;
            if (u > 1.0f || mergedAlpha < u*MAXF)
               B[z] = (B[z]*v1 + A[z]*v2)*(mergedAlpha / u);
            else
               B[z] = B[z]*v1 + A[z]*v2;
        }
        if (B[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] >= (1.0f - EPSILONf))
            B[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = 1.0f;
    }
    B[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = mergedAlpha;
}
//--------------------------------------------------------
// A-under-B compositing operation handling the different DeepSegment
// types.
inline
void DeepPixel::compositeSegmentUnder (Pixelf& B,
                                       float Zf, float Zb, DeepFlags surface_flags,
                                       const Pixelf& A,
                                       float Aalpha,
                                       float Acutout,
                                       float Aspcoverage,
                                       const ChannelSet& color_channels)
{
    // Get acculated visibility (1-B-alpha of last full-spcoverage segment):
    float viz = B[OPENDCX_INTERNAL_NAMESPACE::Chan_Visibility];
    if (viz < EPSILONf)
        return; // zero visibility, don't bother

    // New accumulated B-alpha. Correct for alpha overshooting 1.0 by
    // weighing contribution down by the overshoot amount to avoid any
    // brightening artifacts - usually due to ADDITIVE segments:
    float Ba  = B[OPENDCX_INTERNAL_NAMESPACE::Chan_A] + Aalpha*viz;
    float Bsp = B[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] + Aspcoverage*viz;
    if (Ba > 1.0f)
    {
        // overshoot correction = (A - ((B + A*C) - 1)) / A  or  (A - B - A*C + 1) / A
        const float correction = (Aalpha - B[OPENDCX_INTERNAL_NAMESPACE::Chan_A] - (Aalpha*viz) + 1.0f) / Aalpha;
        Ba = 1.0f;
        //Bsp *= correction;
        viz *= correction;
    }

    // Chan_ACutout contains opacity *with* cutouts, and it's handled explicitly:
    const float Bcuta = B[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout] + Acutout*viz;
    foreach_channel(z, color_channels)
        B[z] += A[z]*viz;
    B[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout] = Bcuta;

    if (surface_flags.isAdditive())
    {
        // Handle saturating additive partial spcoverage:
        if (Aspcoverage > 0.0f && Aspcoverage < 1.0f)
        {
            // Update accumulated visibility and reset accumulated SpCoverage:
            if (Bsp >= (1.0f - EPSILONf))
            {
                B[OPENDCX_INTERNAL_NAMESPACE::Chan_Visibility] = (1.0f - Ba);
                Bsp = 0.0f;
            }
        }
    }
    else
    {
        // Update accumulated visibility for non-additive segments:
        B[OPENDCX_INTERNAL_NAMESPACE::Chan_Visibility] = (1.0f - Ba);
        if (Bsp >= (1.0f - EPSILONf))
            Bsp = 1.0; // keep spcoverage clamped to 1 when non-additive
    }

    // Only min,max Zs if new B-alpha is greater than alpha threshold:
    if (Ba >= EPSILONf)
    {
        if (Zf > 0.0f)
            B[OPENDCX_INTERNAL_NAMESPACE::Chan_ZFront] = std::min(Zf, B[OPENDCX_INTERNAL_NAMESPACE::Chan_ZFront]);
        if (Zb > 0.0f)
            B[OPENDCX_INTERNAL_NAMESPACE::Chan_ZBack ] = std::max(Zb, B[OPENDCX_INTERNAL_NAMESPACE::Chan_ZBack ]);
    }

    // Update output B opacities:
    B[OPENDCX_INTERNAL_NAMESPACE::Chan_A] = Ba;
    B[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage] = Bsp;
}
inline
void DeepPixel::compositeSegmentUnder (Pixelf& B,
                                       float Zf, float Zb, DeepFlags surface_flags,
                                       const Pixelf& A,
                                       const ChannelSet& color_channels)
{
    compositeSegmentUnder(B,
                          Zf, Zb,
                          surface_flags,
                          A,
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_A], /*Aalpha*/
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout], /*Acutout*/
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage],
                          color_channels);
}
inline
void DeepPixel::compositeSegmentUnder (Pixelf& B,
                                       const DeepSegment& segment, const Pixelf& A,
                                       const ChannelSet& color_channels)
{
    compositeSegmentUnder(B,
                          segment.Zf, segment.Zb,
                          segment.surfaceFlags(),
                          A,
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_A], /*Aalpha*/
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_ACutout], /*Acutout*/
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_SpCoverage],
                          color_channels);
}
inline
void DeepPixel::compositeSegmentUnder (Pixelf& B,
                                       const DeepSegment& segment,
                                       const ChannelSet& color_channels)
{
    const Pixelf& A = getSegmentPixel(segment);
    compositeSegmentUnder(B,
                          segment.Zf, segment.Zb,
                          segment.surfaceFlags(),
                          A,
                          A[OPENDCX_INTERNAL_NAMESPACE::Chan_A], /*Aalpha*/
                          (segment.isMatte())?0.0f:A[OPENDCX_INTERNAL_NAMESPACE::Chan_A], /*Acutout*/
                          (segment.hasPartialSpCoverage())?segment.getSpCoverageWeight():1.0f,
                          color_channels);
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DEEPPIXEL_H
