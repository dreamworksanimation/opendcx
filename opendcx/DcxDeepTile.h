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
/// @file DcxDeepTile.h

#ifndef INCLUDED_DCX_DEEPTILE_H
#define INCLUDED_DCX_DEEPTILE_H

//=============================================================================
//
//  class  DeepTile
//
//=============================================================================

#include "DcxPixelTile.h"
#include "DcxDeepPixel.h"
#include "DcxChannelAlias.h"


//-------------------------
//!rst:cpp:begin::
//.. _deeptile_class:
//
//DeepTile
//========
//-------------------------


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//=====================
//
//  class DeepTile
// 
//=====================
//-----------------------------------------------------------------------------
//
//  PixelTile class to manage deep pixels.
//
//  This class is intended for simplifing common pixel-region loops that
//  step from pixel to pixel processing multiple channels simultaneously,
//  vs. using the IlmImfUtil deep classes which seem organized more for
//  per-plane processing and more targeted for deep texture use.
//
//  See the DeepImageTile classes for an example that wraps the IlmImfUtil
//  DeepImage class.
//
//-----------------------------------------------------------------------------

class DCX_EXPORT DeepTile : public PixelTile
{
  public:

    //-------------------------------------------------------------------------------
    //
    // Hints at how the tile should be spatially accessed during writing of pixels.
    // Some memory arrangements allow only sequential writing of the entire tile
    // buffer in order while others want it restricted per-scanline.
    //
    //-------------------------------------------------------------------------------

    enum WriteAccessMode
    {
        WRITE_DISABLED,         // Cannot write to this tile
        WRITE_RANDOM,           // Any x,y pixel can be written to randomly
        WRITE_RANDOM_SCANLINE,  // Scanlines can be written randomly but pixels must be written in order (x argument ignored)
        WRITE_SEQUENTIAL        // Entire tile must be written in sequential order (x & y arguments ignored)
    };


  public:

    //
    // Must provide a ChannelContext at a minimum.
    // Data and display windows are set to invalid value [0 0 -1 -1]
    //

    DeepTile (ChannelContext& channel_ctx,
              WriteAccessMode write_access_mode=WRITE_DISABLED,
              bool yAxisUp=true);

    //
    // Gets resolution info and channel set from header.
    // The set of ChannelAliases is used to construct a map of ChannelIdx's to
    // ChannelAliases.  If multiple input ChannelAliases have the same ChannelIdx
    // destination only the first one is accepted and the others are ignored.
    //

    DeepTile (const Imf::Header&,
              ChannelContext& channel_ctx,
              WriteAccessMode write_access_mode=WRITE_DISABLED,
              bool yAxisUp=true);

    //
    // Assigns resolution and channel set.
    // The set of ChannelAliases is used to construct a map of ChannelIdx's to
    // ChannelAliases.  If multiple input ChannelAliases have the same ChannelIdx
    // destination only the first one is accepted and the others are ignored.
    //
    // display_window is only used to set the window-top reference.
    //

    DeepTile (const IMATH_NAMESPACE::Box2i& display_window,
              const IMATH_NAMESPACE::Box2i& data_window,
              bool sourceWindowsYup,
              const ChannelSet& channels,
              ChannelContext& channel_ctx,
              WriteAccessMode write_access_mode=WRITE_DISABLED,
              bool yAxisUp=true);
    DeepTile (const IMATH_NAMESPACE::Box2i& data_window,
              int top_reference,
              bool sourceWindowsYAxisUp,
              const ChannelSet& channels,
              ChannelContext& channel_ctx,
              WriteAccessMode write_access_mode=WRITE_DISABLED,
              bool yAxisUp=true);


    //
    // Assign the active ChannelSet.
    // If spmask or flag channels are in the set this will update the spmask channel
    // count and the flags channel.
    //

    /*virtual*/ void setChannels (const ChannelSet& channels,
                                  bool force=false);


    //
    // Returns the number of deep samples at pixel x,y.
    //

    virtual size_t getNumSamplesAt (int x, int y) const=0;

    //
    // Reads deep samples from a pixel-space location (x, y) into a deep pixel.
    // If xy is out of bounds the deep pixel is left empty and false is returned.
    // Must be implemented by subclasses.
    //

    virtual bool getDeepPixel (int x,
                               int y,
                               Dcx::DeepPixel& pixel) const=0;

    virtual bool getSampleMetadata (int x,
                                    int y,
                                    size_t sample,
                                    Dcx::DeepMetadata& metadata) const=0;


    //
    // Writes a DeepPixel to a pixel-space location (x, y) in the deep channels.
    // If xy is out of bounds or the tile can't be written to, the deep pixel
    // is left empty and false is returned.
    // x or y arguments might be ignored depending on write access mode.
    //
    // Base class returns false.
    //

    virtual bool setDeepPixel (int x,
                               int y,
                               const Dcx::DeepPixel& pixel);


    //
    // Writes an empty DeepPixel (0 samples) at a pixel-space location (x, y).
    // Returns false if x,y is out of bounds or the tile can't be written to.
    // x or y arguments might be ignored depending on write access mode.
    //
    // Base class returns false.
    //

    virtual bool clearDeepPixel (int x,
                                 int y);


    //
    // Hints at how the tile should be spatially accessed during writing of
    // pixels.
    //

    WriteAccessMode writeAccessMode () const;
    bool            writable() const;


    //
    // Does the tile contain channels for sample metadata storage?
    //

    bool    hasSpMasks () const;
    bool    hasFlags () const;



  protected:
    //
    // Copy constructor only for subclasses.
    //
    DeepTile (const DeepTile&);


    WriteAccessMode     m_write_access_mode;    // Spatial write-access mode
    Dcx::ChannelIdx     m_spmask_channel[2];    // SpMask8 channels - active if not Chan_Invalid
    Dcx::ChannelIdx     m_flags_channel;        // Flags channel - active if not Chan_Invalid

};



//--------------
//!rst:cpp:end::
//--------------

//-----------------
// Inline Functions
//-----------------
inline bool DeepTile::setDeepPixel (int, int, const Dcx::DeepPixel&) { return false; }
inline bool DeepTile::clearDeepPixel (int, int) { return false; }
//
inline DeepTile::WriteAccessMode DeepTile::writeAccessMode () const { return m_write_access_mode; }
inline bool DeepTile::writable () const { return (m_write_access_mode > WRITE_DISABLED); }
//
inline bool DeepTile::hasSpMasks () const { return (m_spmask_channel[0] != Dcx::Chan_Invalid && m_spmask_channel[1] != Dcx::Chan_Invalid); }
inline bool DeepTile::hasFlags () const { return (m_flags_channel != Dcx::Chan_Invalid); }


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DEEPTILE_H
