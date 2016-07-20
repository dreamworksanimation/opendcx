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

//-----------------------------------------------------------------------------
//
//  class  DeepTransform
//
//-----------------------------------------------------------------------------

#include "DcxDeepTile.h"

#ifdef __ICC
// disable icc remark #1572: 'floating-point equality and inequality comparisons are unreliable'
//   this is coming from OpenEXR/half.h...
#   pragma warning(disable:2557)
#endif
#include <OpenEXR/ImathMatrix.h>


//-----------------------------------------------------------------------------------------
// TODO: Seems like these convenience functions should be in Imath where M_PI is defined,
// or from a standard lib. Suggestions?
#include <OpenEXR/ImathPlatform.h> // for M_PI

template <class T>
T radians (T degrees) { return degrees*(M_PI/180); }
template <class T>
T degrees (T radians) { return radians*(180/M_PI); }
//-----------------------------------------------------------------------------------------


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//-----------------------------------------------------------------------------
//
// class DeepTransform
//      Supports 2D transforms with subpixel mask resampling.
//
//      (TODO:) Will also handle limited Z transforms that shift / scale the
//              deep samples in depth.
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


  public:

    DeepTransform (int super_sampling=4,
                   FilterMode filter_mode=FILTER_BOX);
    DeepTransform (const IMATH_NAMESPACE::M44f&,
                   int super_sampling=4,
                   FilterMode filter_mode=FILTER_BOX);
    virtual ~DeepTransform ();


    //
    // Filter mode used during sample().
    //

    bool filterMode() const;


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
    void    scale (const IMATH_NAMESPACE::V2f& v);
    void    scaleZ (float sz);

    void    translate (float tx, float ty);
    void    translate (const IMATH_NAMESPACE::V2f& v);
    void    translateZ (float tz);


    //
    // Transform through forward matrix.
    //

    void         transform (float x,
                            float y,
                            float& outX,
                            float& outY,
                            const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::V2f   transform (const IMATH_NAMESPACE::V2f&,
                            const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::Box2i transform (const IMATH_NAMESPACE::Box2i&,
                            const IMATH_NAMESPACE::Box2i* clamp_to=0) const;

    //
    // Transform through inverse matrix.
    //

    void         backTransform (float x,
                                float y,
                                float& outX,
                                float& outY,
                                const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::V2f   backTransform (const IMATH_NAMESPACE::V2f&,
                                const IMATH_NAMESPACE::Box2i* clamp_to=0) const;
    IMATH_NAMESPACE::Box2i backTransform (const IMATH_NAMESPACE::Box2i&,
                                const IMATH_NAMESPACE::Box2i* clamp_to=0) const;

    //
    // Transform with provided matrix.
    //

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


    //
    // Clamp x/y values to a bounding-box.
    //

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

    //
    // Sample input deep pixels with subpixel-mask resampling.
    //

    virtual void sample (int outX,
                         int outY,
                         const DeepTile& deep_in_tile,
                         Dcx::DeepPixel& out_pixel);


    virtual void transformTile (const DeepTile& in_tile,
                                DeepTile& out_tile);


  protected:

    IMATH_NAMESPACE::M44f   m_matrix;           // Scale/rot/trans matrix
    IMATH_NAMESPACE::M44f   m_imatrix;          // Inverse scale/rot/trans matrix
    bool                    m_updated;          // Is inverse matrix up to date?
    float                   m_zTranslate;       // Separate from matrix for convenience
    float                   m_zScale;           // Separate from matrix for convenience
    bool                    m_filter_mode;      // What kind of filtering to perform
    int                     m_ss_factor;        // Sampling rate (can be different than subpixel mask res)

};



//-----------------
// Inline Functions
//-----------------

inline
bool DeepTransform::filterMode () const { return m_filter_mode; }
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
void DeepTransform::scale (const IMATH_NAMESPACE::V2f& v) { m_matrix.scale(IMATH_NAMESPACE::V3f(v.x, v.y, 1.0f)); m_updated = false; }
inline
void DeepTransform::scaleZ (float sz) { m_zScale = sz; }
//
inline
void DeepTransform::translate (float tx, float ty) { m_matrix.translate(IMATH_NAMESPACE::V3f(tx, ty, 0.0f)); m_updated = false; }
inline
void DeepTransform::translate (const IMATH_NAMESPACE::V2f& v) { m_matrix.translate(IMATH_NAMESPACE::V3f(v.x, v.y, 0.0f)); m_updated = false; }
inline
void DeepTransform::translateZ (float tz) { m_zTranslate = tz; }
//
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
//
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
//
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
