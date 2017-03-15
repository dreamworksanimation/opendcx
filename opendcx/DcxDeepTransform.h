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
/// @file DcxDeepTransform.h

#ifndef INCLUDED_DCX_DEEPTRANSFORM_H
#define INCLUDED_DCX_DEEPTRANSFORM_H

//=============================================================================
//
//  class  DeepTransform
//  enum   DeepTransform::FilterMode
//  struct DeepTransform::SegmentRef
//
//=============================================================================

#include "DcxDeepTile.h"

#ifdef __ICC
// disable icc remark #1572: 'floating-point equality and inequality comparisons are unreliable'
//   this is coming from OpenEXR/half.h...
#   pragma warning(disable:2557)
#endif
#include <OpenEXR/ImathMatrix.h>

#include <OpenEXR/IlmThreadMutex.h> // for matrix-inversion lock


//-------------------------
//!rst:cpp:begin::
//.. _deeptransform_class:
//
//DeepTransform
//=============
//-------------------------


//=============================================================================
// TODO: Seems like these convenience functions should be in Imath where M_PI
//      is defined, or from a standard lib. Suggestions?
#include <OpenEXR/ImathPlatform.h> // for M_PI

template <class T>
T radians (T degrees) { return degrees*(M_PI/180); }
template <class T>
T degrees (T radians) { return radians*(180/M_PI); }
//=============================================================================


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//=========================
//
//  class DeepTransform
// 
//=========================
//-----------------------------------------------------------------------------
//
//  Supports 2D transforms with subpixel mask resampling.
//
//-----------------------------------------------------------------------------

class DCX_EXPORT DeepTransform
{
  public:

    //
    // Filter modes used during sample().
    //

    enum FilterMode
    {
        FILTER_NEAREST,     //
        FILTER_BOX          //
    };


    //
    // Sampling rate for subpixel mask reconstruction
    //

    enum SuperSamplingMode
    {
        SS_RATE_1,
        SS_RATE_2,
        SS_RATE_4,
        SS_RATE_8,
        SS_RATE_16,
        SS_RATE_32,
        //
        SS_RATE_MAX
    };


    //------------------------------------------------------------------------
    //  struct SampleOffset
    //      Subpixel offsets and weight
    //------------------------------------------------------------------------

    struct SampleOffset
    {
        float   dx, dy, wt;
    };

    typedef std::vector<SampleOffset>       SampleOffsetTable;
    typedef std::vector<SampleOffsetTable>  SampleOffsetTableList;


    //------------------------------------------------------------------------
    //  struct DeepTransform::SegmentRef
    //      Used to keep track of the segments from multiple input deep pixels
    //      possibly contributing to one output pixel.
    //------------------------------------------------------------------------

    struct SegmentRef
    {
        int32_t     x, y;
        int32_t     segment;
        float       Zf;
        SpMask8     spmask;

        SegmentRef (int32_t _x, int32_t _y, int32_t _segment,
                    float _Zf,
                    const SpMask8& _spmask);

        bool operator < (const SegmentRef& b) const;
    };


  public:

    DeepTransform (SuperSamplingMode super_sampling=SS_RATE_4,
                   FilterMode filter_mode=FILTER_BOX);

    DeepTransform (const IMATH_NAMESPACE::M44f&,
                   SuperSamplingMode super_sampling=SS_RATE_4,
                   FilterMode filter_mode=FILTER_BOX);

    virtual ~DeepTransform ();


    //
    // Filter mode used during sample().
    //

    FilterMode  filterMode () const;
    void        setFilterMode (FilterMode);


    //
    // Supersampling mode and rate value used during sample().
    //

    SuperSamplingMode   superSamplingMode () const;
    virtual uint32_t    superSamplingRate () const;
    virtual uint32_t    superSamplingWidth () const;

    //
    // Set the supersampling mode and fill the supersample offset table(s).
    // Default implementation does uniform distribution of subpixel sample
    // offsets.
    //

    virtual void        setSuperSamplingMode (SuperSamplingMode,
                                              bool force=false);

    //
    // Sample offset table access.
    //

    size_t                              numSuperSamplingTables() const;
    const SampleOffsetTable&            getSuperSamplingTable (size_t i=0) const;
    virtual const SampleOffsetTable&    getSuperSamplingTable (int pixel_x, int pixel_y) const;


    //
    // Get/set/change the matrix.
    //

    const IMATH_NAMESPACE::M44f&  matrix () const;
    const IMATH_NAMESPACE::M44f&  imatrix();

    void    setMatrix (const IMATH_NAMESPACE::M44f& m);

    void    makeIdentity ();
    void    multiply (const IMATH_NAMESPACE::M44f& m);
    void    preMultiply (const IMATH_NAMESPACE::M44f& m);

    void    rotate (float radians);

    void    scale (float s);
    void    scale (float sx, float sy);
    void    scale (const IMATH_NAMESPACE::V2f& s);

    void    translate (float tx, float ty);
    void    translate (const IMATH_NAMESPACE::V2f& t);


    //-------------------------------------------
    // Forward-transform xy coord through matrix.
    //-------------------------------------------

    void         transform (float x,
                            float y,
                            float& outX,
                            float& outY,
                            const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::V2f   transform (const IMATH_NAMESPACE::V2f&,
                                      const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::Box2i transform (const IMATH_NAMESPACE::Box2i&,
                                      const IMATH_NAMESPACE::Box2i* clamp_to=0) const;


    //---------------------------------------------------
    // Reverse-transform xy coord through inverse matrix.
    //---------------------------------------------------

    void         backTransform (float x,
                                float y,
                                float& outX,
                                float& outY,
                                const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::V2f   backTransform (const IMATH_NAMESPACE::V2f&,
                                          const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::Box2i backTransform (const IMATH_NAMESPACE::Box2i&,
                                          const IMATH_NAMESPACE::Box2i* clamp_to=0) const;


    //-----------------------------------------
    // Transform xy coord with provided matrix.
    //-----------------------------------------

    static void transform (float x,
                           float y,
                           const IMATH_NAMESPACE::M44f& m,
                           float& outX,
                           float& outY);
    static void transform (const IMATH_NAMESPACE::V2f& in,
                           const IMATH_NAMESPACE::M44f& m,
                           IMATH_NAMESPACE::V2f& out);
    static void transform (const IMATH_NAMESPACE::Box2i& in,
                           const IMATH_NAMESPACE::M44f& m,
                           IMATH_NAMESPACE::Box2i& out);


    //------------------------------------
    // Clamp x/y values to a bounding-box.
    //------------------------------------

    static void clampToBbox (float& x,
                             float& y,
                             const IMATH_NAMESPACE::Box2i& clamp_bbox);
    static void clampToBbox (int& x,
                             int& y,
                             const IMATH_NAMESPACE::Box2i& clamp_bbox);
    static void clampToBbox (IMATH_NAMESPACE::V2f& v,
                             const IMATH_NAMESPACE::Box2i& clamp_bbox);
    static void clampToBbox (IMATH_NAMESPACE::V2i& v,
                             const IMATH_NAMESPACE::Box2i& clamp_bbox);
    static void clampToBbox (IMATH_NAMESPACE::Box2i& bbox,
                             const IMATH_NAMESPACE::Box2i& clamp_bbox);


    //------------------------------------
    // DeepPixel sampling methods
    //------------------------------------

    //
    // Sample all input deep pixels contributing to output deep pixel outX/outY.
    //

    virtual void sample (int outX, int outY,
                         const DeepTile& deep_in_tile,
                         const ChannelSet& sample_channels,
                         Dcx::DeepPixel& deep_out_pixel);

    //
    // Transform a single input deep pixel and resample it to an output deep tile.
    //

    virtual void transformPixel (const DeepTile& deep_in_tile,
                                 int inX, int inY,
                                 DeepTile& deep_out_tile);


    //
    // Transform all the pixels of a tile.
    //

    virtual void transformTile (const DeepTile& deep_in_tile,
                                DeepTile& deep_out_tile);


    //
    //  Sample deep pixels contributing to output deep pixel outX/outY from
    //  a set of preselected input segments, with subpixel-mask resampling.
    //
    //  Note: deep_out_pixel is not cleared by this method, allowing multiple
    //  input DeepPixels to be combined into it.
    //
    //  Note: the segment ref list should be z-sorted prior to calling this
    //  method to allow the generated output segments to combine more readily:
    //      std::sort(sample_segments.begin(), sample_segments.end());
    //
    //  This method is called by the other sample/transform methods and
    //  does the actual work of resampling the segments.
    //

    void    sampleSegments (const DeepTile& deep_in_tile,
                            const std::vector<SegmentRef>& sample_segments,
                            const ChannelSet& sample_channels,
                            int outX, int outY,
                            Dcx::DeepPixel& deep_out_pixel);


  protected:

    //
    // Fill the supersample offset table(s).
    // Default implementation does uniform distribution of subpixel sample
    // offsets.
    //

    virtual void    fillSuperSamplingTables (uint32_t ss_width,
                                             uint32_t num_tables=1);


    // Assigned vars:
    IMATH_NAMESPACE::M44f   m_matrix;           // Scale/rot/trans matrix
    SuperSamplingMode       m_ss_mode;          // Sampling rate mode for subpixel mask reconstruction
    FilterMode              m_filter_mode;      // What kind of filtering to perform
    SampleOffsetTableList   m_sp_offset_tables; // List of super-sample subpixel offset tables

    // Derived vars:
    IMATH_NAMESPACE::M44f   m_imatrix;          // Inverse scale/rot/trans matrix
    bool                    m_updated;          // Is inverse matrix up to date?

    ILMTHREAD_NAMESPACE::Mutex m_mutex;         // For locking matrix inversion and sample-table build

};



//----------
//!rst:cpp:end::
//----------

//-----------------
// Inline Functions
//-----------------

inline
DeepTransform::SegmentRef::SegmentRef (int32_t _x, int32_t _y,
                                       int32_t _segment,
                                       float _Zf,
                                       const SpMask8& _spmask) :
    x(_x), y(_y),
    segment(_segment),
    Zf(_Zf),
    spmask((_spmask == SpMask8::zeroCoverage)?SpMask8::fullCoverage:_spmask)
{
    //
}
inline
bool DeepTransform::SegmentRef::operator < (const SegmentRef& b) const { return (Zf < b.Zf); }
//--------------------------------------------------------
inline
DeepTransform::FilterMode DeepTransform::filterMode () const { return m_filter_mode; }
inline
void DeepTransform::setFilterMode (DeepTransform::FilterMode m) { m_filter_mode = m; }
inline
DeepTransform::SuperSamplingMode DeepTransform::superSamplingMode () const { return m_ss_mode; }
inline
size_t DeepTransform::numSuperSamplingTables() const { return m_sp_offset_tables.size(); }
inline
const DeepTransform::SampleOffsetTable& DeepTransform::getSuperSamplingTable (size_t i) const { return m_sp_offset_tables[i]; }
inline
const IMATH_NAMESPACE::M44f& DeepTransform::matrix () const { return m_matrix; }
inline
void DeepTransform::setMatrix (const IMATH_NAMESPACE::M44f& m) { m_matrix = m; m_updated = false; }
//
inline
void DeepTransform::makeIdentity () { m_matrix.makeIdentity(); m_imatrix.makeIdentity(); m_updated = true; }
inline
void DeepTransform::multiply (const IMATH_NAMESPACE::M44f& m) { m_matrix *= m; m_updated = false; }
inline
void DeepTransform::preMultiply (const IMATH_NAMESPACE::M44f& m) { m_matrix = m * m_matrix; m_updated = false; }
//
inline
void DeepTransform::rotate (float radians) { m_matrix.rotate(IMATH_NAMESPACE::V3f(0.0f, 0.0f, radians)); m_updated = false; }
//
inline
void DeepTransform::scale (float s) { m_matrix.scale(IMATH_NAMESPACE::V3f(s, s, 1.0f)); m_updated = false; }
inline
void DeepTransform::scale (float sx, float sy) { m_matrix.scale(IMATH_NAMESPACE::V3f(sx, sy, 1.0f)); m_updated = false; }
inline
void DeepTransform::scale (const IMATH_NAMESPACE::V2f& s) { m_matrix.scale(IMATH_NAMESPACE::V3f(s.x, s.y, 1.0f)); m_updated = false; }
//
inline
void DeepTransform::translate (float tx, float ty) { m_matrix.translate(IMATH_NAMESPACE::V3f(tx, ty, 0.0f)); m_updated = false; }
inline
void DeepTransform::translate (const IMATH_NAMESPACE::V2f& t) { m_matrix.translate(IMATH_NAMESPACE::V3f(t.x, t.y, 0.0f)); m_updated = false; }
//----------------------------------
inline
/*static*/
void DeepTransform::clampToBbox (float& x, float& y, const IMATH_NAMESPACE::Box2i& clamp_bbox)
{
    x = std::max(float(clamp_bbox.min.x), std::min(x, float(clamp_bbox.max.x)));
    y = std::max(float(clamp_bbox.min.y), std::min(y, float(clamp_bbox.max.y)));
}
inline
/*static*/
void DeepTransform::clampToBbox (int& x, int& y, const IMATH_NAMESPACE::Box2i& clamp_bbox)
{
    x = std::max(clamp_bbox.min.x, std::min(x, clamp_bbox.max.x));
    y = std::max(clamp_bbox.min.y, std::min(y, clamp_bbox.max.y));
}
inline
/*static*/
void DeepTransform::clampToBbox (IMATH_NAMESPACE::V2f& v, const IMATH_NAMESPACE::Box2i& clamp_bbox) { clampToBbox(v.x, v.y, clamp_bbox); }
inline
/*static*/
void DeepTransform::clampToBbox (IMATH_NAMESPACE::V2i& v, const IMATH_NAMESPACE::Box2i& clamp_bbox) { clampToBbox(v.x, v.y, clamp_bbox); }
inline
/*static*/
void DeepTransform::clampToBbox (IMATH_NAMESPACE::Box2i& bbox, const IMATH_NAMESPACE::Box2i& clamp_bbox) { clampToBbox(bbox.min, clamp_bbox); clampToBbox(bbox.max, clamp_bbox); }
//----------------------------------
inline
/*static*/
void DeepTransform::transform (float x, float y, const IMATH_NAMESPACE::M44f& m, float& outX, float& outY)
{
    const IMATH_NAMESPACE::V3f p = IMATH_NAMESPACE::V3f(x, y, 0.0f) * m;
    outX = p.x;
    outY = p.y;
}
inline
/*static*/
void DeepTransform::transform (const IMATH_NAMESPACE::V2f& in, const IMATH_NAMESPACE::M44f& m, IMATH_NAMESPACE::V2f& out)
{
    DeepTransform::transform(in.x, in.y, m, out.x, out.y);
}
inline
void DeepTransform::transform (float x, float y, float& outX, float& outY, const IMATH_NAMESPACE::Box2i* clamp_to) const
{
    DeepTransform::transform(x, y, this->matrix(), outX, outY);
    if (clamp_to)
        clampToBbox(outX, outY, *clamp_to);
}
inline
IMATH_NAMESPACE::V2f DeepTransform::transform (const IMATH_NAMESPACE::V2f& v, const IMATH_NAMESPACE::Box2i* clamp_to) const
{
    IMATH_NAMESPACE::V2f out;
    DeepTransform::transform(v, this->matrix(), out);
    if (clamp_to)
        clampToBbox(out, *clamp_to);
    return out;
}
inline
IMATH_NAMESPACE::Box2i DeepTransform::transform (const IMATH_NAMESPACE::Box2i& bbox, const IMATH_NAMESPACE::Box2i* clamp_to) const
{
    IMATH_NAMESPACE::Box2i out;
    DeepTransform::transform(bbox, this->matrix(), out);
    if (clamp_to)
        clampToBbox(out, *clamp_to);
    return out;
}
//----------------------------------
inline
void DeepTransform::backTransform (float x, float y, float& outX, float& outY, const IMATH_NAMESPACE::Box2i* clamp_to) const
{
    DeepTransform::transform(x, y, (const_cast<DeepTransform*>(this))->imatrix(), outX, outY);
    if (clamp_to)
        clampToBbox(outX, outY, *clamp_to);
}
inline
IMATH_NAMESPACE::V2f DeepTransform::backTransform (const IMATH_NAMESPACE::V2f& v, const IMATH_NAMESPACE::Box2i* clamp_to) const
{
    IMATH_NAMESPACE::V2f out;
    DeepTransform::transform(v, (const_cast<DeepTransform*>(this))->imatrix(), out);
    if (clamp_to)
        clampToBbox(out, *clamp_to);
    return out;
}
inline
IMATH_NAMESPACE::Box2i DeepTransform::backTransform (const IMATH_NAMESPACE::Box2i& bbox, const IMATH_NAMESPACE::Box2i* clamp_to) const
{
    IMATH_NAMESPACE::Box2i out;
    DeepTransform::transform(bbox, (const_cast<DeepTransform*>(this))->imatrix(), out);
    if (clamp_to)
        clampToBbox(out, *clamp_to);
    return out;
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DEEPTRANSFORM_H
