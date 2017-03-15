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
//  Utility functions & classes to translate between Nuke's DDImage deep
//  structures and OpenDCX's.
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
// Assign/get the DD::Image::Channels that are appropriate for OpenDCX.
//
// This calls DD::Image::getChannel() with 'sort=false' so that Nuke does
// not reorder the channels in the 'spmask' layer - we always want the order
// to be sp1/sp2/flags so that ChannelKnobs display correctly by default.
//
// To help ensure this there's a static dummy object that calls this method
// so that the layer and channels are assigned in Nuke before any Op
// constructors are called.
//

DCX_EXPORT
void
dcxGetSpmaskChannels (DD::Image::Channel& sp1,
                      DD::Image::Channel& sp2,
                      DD::Image::Channel& flags);


//
// Add new DD::Image::Channel by getting its full name and looking it
// up in the Dcx::ChannelContext.
// This may create a new Dcx::ChannelIdx.
//

DCX_EXPORT
Dcx::ChannelIdx
dcxAddChannel (DD::Image::Channel z);


//
// Get a Dcx::ChannelIdx from a DD::Image::Channel.
//
// This relies dcxChannelSet() being called beforehand to convert the sets and
// create the mappings!
//

DCX_EXPORT
Dcx::ChannelIdx
dcxChannel (DD::Image::Channel z);


//
// Convert a DD::Image::ChannelSet to a Dcx::ChannelSet.
//
// This stores previously converted ChannelSets and returns quickly if the same
// set is requested again.
//

DCX_EXPORT
Dcx::ChannelSet
dcxChannelSet (const DD::Image::ChannelSet& in);


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
                         const DD::Image::Channel spmask_channel0,
                         const DD::Image::Channel spmask_channel1,
                         const DD::Image::Channel flags_channel,
                         Dcx::DeepPixel& out);


//
// Copy the contents of a DD::Image::DeepPixel sample to a Dcx::DeepSegment
// and optionally a Dcx::Pixelf, returning true on success.
//
// Note that the DeepSegment's channel-array index is left unassigned (-1)
// since the copy is done outside the scope of a Dcx::DeepPixel.
//

DCX_EXPORT
bool
dcxCopyDDImageDeepSample (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          const DD::Image::Channel spmask_channel0,
                          const DD::Image::Channel spmask_channel1,
                          const DD::Image::Channel flags_channel,
                          Dcx::DeepSegment& segment_out,
                          Dcx::Pixelf* pixel_out);


//
// Extract subpixel mask and flags metadata from a DD::Image::DeepPixel sample.
//

DCX_EXPORT
void
dcxGetDeepSampleMetadata (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          const DD::Image::Channel spmask_channel0,
                          const DD::Image::Channel spmask_channel1,
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
    // Construct a Dcx::DeepTile from DDImage structures.
    // The DeepTile's displayWindow is copied from format while its dataWindow
    // comes from bbox.
    //
    // No pixel data is copied.
    //

    DDImageDeepPlane (const DD::Image::Box& format,
                      const DD::Image::Box& bbox,
                      const DD::Image::ChannelSet& channels,
                      const DD::Image::Channel spmask_channel0,
                      const DD::Image::Channel spmask_channel1,
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
                                   Dcx::DeepPixel& pixel) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata) const;


  protected:
    // To pass to dcxCopyDDImageDeepPixel():
    DD::Image::Channel  m_spmask_channel[2];
    DD::Image::Channel  m_flags_channel;

};


//--------------------------------------------------------------------------------


class DDImageDeepInputPlane : public DDImageDeepPlane
{
  public:

    //
    // Copy resolution and channel info from input DD::Image::Format and
    // DD::Image::DeepPlane.  The Format is required to define the Dcx::DeepTile's
    // displayWindow while the dataWindow comes from the DeepPlane.
    //
    // No pixel data is copied.
    //

    DDImageDeepInputPlane (const DD::Image::Box& format,
                           const DD::Image::DeepPlane* deep_in_plane,
                           const DD::Image::Channel spmask_channel0,
                           const DD::Image::Channel spmask_channel1,
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
                                   Dcx::DeepPixel& pixel) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata) const;


  protected:
    const DD::Image::DeepPlane*     m_in_plane;

};


//--------------------------------------------------------------------------------


class DDImageDeepOutputPlane : public DDImageDeepPlane
{
  public:

    //
    // Copy resolution and channel info from input DD::Image::Format and
    // DD::Image::DeepPlane.  The Format is required to define the Dcx::DeepTile's
    // displayWindow while the dataWindow comes from the DeepPlane.
    //
    // No pixel data is copied.
    //

    DDImageDeepOutputPlane (const DD::Image::Box& format,
                            DD::Image::DeepOutputPlane* deep_out_plane,
                            const DD::Image::Channel spmask_channel0,
                            const DD::Image::Channel spmask_channel1,
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
                                   Dcx::DeepPixel& pixel) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata) const;


    //
    // Writes a DeepPixel to the DeepOutputPlane.
    // Write access is sequential for a DD::Image::DeepOutputPlane so
    // x/y args are not required
    //

    void setDeepPixelSequential (const Dcx::DeepPixel& pixel);

    //
    // Writes an empty DeepPixel (0 samples) to the DeepOutputPlane.
    // Write access is sequential for a DD::Image::DeepOutputPlane so
    // x/y args are not required
    //

    void clearDeepPixelSequential ();


  protected:
    DD::Image::DeepOutputPlane* m_out_plane;

};


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DDIMAGEADAPTER_H
