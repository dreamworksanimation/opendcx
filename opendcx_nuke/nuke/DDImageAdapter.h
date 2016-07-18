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
/// @file DDImageAdapter.h

#ifndef INCLUDED_DCX_DDIMAGEADAPTER_H
#define INCLUDED_DCX_DDIMAGEADAPTER_H

//-----------------------------------------------------------------------------
//
//  class  DDImageDeepPlane
//  class  DDImageDeepInputPlane
//  class  DDImageDeepOutputPlane
//
//-----------------------------------------------------------------------------

#include <OpenDCX/DcxDeepPixel.h>
#include <OpenDCX/DcxDeepTile.h>

#include <OpenEXR/ImathMatrix.h>

#include <DDImage/DeepPixel.h>
#include <DDImage/DeepPlane.h>
#include <DDImage/Matrix4.h>


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

//--------------------------------------------------------------------------------

//
// Convert a DD::Image::ChannelSet to a Dcx::ChannelSet.
//
// This stores previously converted ChannelSets and returns quickly if the same
// set is requested again.
//

DCX_EXPORT
Dcx::ChannelSet
dcxChannelSet (const DD::Image::ChannelSet& in);


//
// Convert a DD::Image::ChannelSet to a Dcx::ChannelAliasSet.
//

DCX_EXPORT
Dcx::ChannelAliasSet
dcxChannelAliasSet (const DD::Image::ChannelSet& in);


//
// Get a Dcx::ChannelIdx from a DD::Image::Channel.
//
// This relies dcxChannelSet() being called beforehand to convert the sets and
// create the mappings!
//

DCX_EXPORT
Dcx::ChannelIdx
dcxChannel (DD::Image::Channel z);

//--------------------------------------------------------------------------------

DCX_EXPORT
Imath::M44f
dcxMatrix4ToM44f(const DD::Image::Matrix4& m);

//--------------------------------------------------------------------------------

//
// Copy the contents of a DD::Image::DeepPixel to a Dcx::DeepPixel.
//

DCX_EXPORT
void
dcxCopyDDImageDeepPixel (const DD::Image::DeepPixel& in,
                         Dcx::SpMaskMode spmask_mode,
                         const std::vector<DD::Image::Channel>& spmask_chan_list,
                         const DD::Image::Channel flags_channel,
                         Dcx::DeepPixel& out);


//
// Copy the contents of a DD::Image::DeepPixel sample to a Dcx::DeepSegment
// and Dcx::Pixelf, returning true on success.
// Note that the DeepSegment's channel-array index is left unassigned (-1)
// since the copy is done outside the scope of a DeepPixel.
//

DCX_EXPORT
bool
dcxCopyDDImageDeepSample (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          Dcx::SpMaskMode spmask_mode,
                          const std::vector<DD::Image::Channel>& spmask_chan_list,
                          const DD::Image::Channel flags_channel,
                          Dcx::DeepSegment& segment_out,
                          Dcx::Pixelf* pixel_out);


//
// Extract subpixel mask and flags metadata from input 'spmask' layer.
//

DCX_EXPORT
void
dcxGetDeepSampleMetadata (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          Dcx::SpMaskMode spmask_mode,
                          const std::vector<DD::Image::Channel>& spmask_chan_list,
                          const DD::Image::Channel flags_channel,
                          Dcx::DeepMetadata& metadata_out);


//--------------------------------------------------------------------------------


//
// Interface wrappers for DD::Image::DeepPlane & DD::Image::DeepOutputPlane
//

class DDImageDeepPlane : public Dcx::DeepTile
{
  public:

    //
    // Copy resolution and channel info from another DeepTile.
    // No pixel data is copied.
    //

    DDImageDeepPlane (const DD::Image::Box& format,
                      const DD::Image::Box& bbox,
                      const DD::Image::ChannelSet& channels,
                      Dcx::SpMaskMode spmask_mode,
                      const std::vector<DD::Image::Channel>& spmask_chan_list,
                      const DD::Image::Channel flags_channel,
                      WriteAccessMode write_access_mode=WRITE_DISABLED);


    //
    // Returns the number of deep samples at pixel x,y.
    //

    /*virtual*/ size_t getNumSamplesAt (int x, int y) const;


    //
    // Reads deep samples from a pixel-space location (x, y) into a deep pixel.
    // If xy is out of bounds the deep pixel is left empty and false is returned.
    //

    /*virtual*/ bool getDeepPixel (int x,
                                   int y,
                                   Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   ) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                        , bool debug=false
#endif
                                        ) const;


  protected:
    // To pass to dcxCopyDDImageDeepPixel():
    Dcx::SpMaskMode                     m_spmask_mode;
    std::vector<DD::Image::Channel>     m_spmask8_chan_list;
    DD::Image::Channel                  m_flags_channel;

};


//--------------------------------------------------------------------------------


class DDImageDeepInputPlane : public DDImageDeepPlane
{
  public:

    //
    // Copy resolution and channel info from another DeepTile.
    // No pixel data is copied.
    //

    DDImageDeepInputPlane (const DD::Image::Box& format,
                           const DD::Image::DeepPlane* deep_in_plane,
                           Dcx::SpMaskMode spmask_mode,
                           const std::vector<DD::Image::Channel>& spmask_chan_list,
                           const DD::Image::Channel flags_channel);


    //
    // Returns the number of deep samples at pixel x,y.
    //

    /*virtual*/ size_t getNumSamplesAt (int x, int y) const;


    //
    // Reads deep samples from a pixel-space location (x, y) into a deep pixel.
    // If xy is out of bounds the deep pixel is left empty and false is returned.
    //

    /*virtual*/ bool getDeepPixel (int x,
                                   int y,
                                   Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   ) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                        , bool debug=false
#endif
                                        ) const;


  protected:
    const DD::Image::DeepPlane*     m_in_plane;

};


//--------------------------------------------------------------------------------


class DDImageDeepOutputPlane : public DDImageDeepPlane
{
  public:

    //
    // Copy resolution and channel info from another DeepTile.
    // No pixel data is copied.
    //

    DDImageDeepOutputPlane (const DD::Image::Box& format,
                            DD::Image::DeepOutputPlane* deep_out_plane,
                            Dcx::SpMaskMode spmask_mode,
                            const std::vector<DD::Image::Channel>& spmask_chan_list,
                            const DD::Image::Channel flags_channel);


    //
    // Returns the number of deep samples at pixel x,y.
    //

    /*virtual*/ size_t getNumSamplesAt (int x, int y) const;


    //
    // Reads deep samples from a pixel-space location (x, y) into a deep pixel.
    // If xy is out of bounds the deep pixel is left empty and false is returned.
    //

    /*virtual*/ bool getDeepPixel (int x,
                                   int y,
                                   Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   ) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                        , bool debug=false
#endif
                                        ) const;


    //
    // Writes a DeepPixel to the DeepOutputPlane.
    // Write access is sequential so both x/y args are ignored.
    //

    /*virtual*/ bool setDeepPixel (int,/*x ignored*/
                                   int,/*y ignored*/
                                   const Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   );

    //
    // Writes an empty DeepPixel (0 samples) to the DeepOutputPlane.
    // Write access is sequential so both x/y args are ignored.
    //

    /*virtual*/ bool clearDeepPixel (int,/*x ignored*/
                                     int /*y ignored*/
#ifdef DCX_DEBUG_DEEPTILE
                                     , bool debug=false
#endif
                                     );


  protected:
    DD::Image::DeepOutputPlane*     m_out_plane;

};


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DDIMAGEADAPTER_H
