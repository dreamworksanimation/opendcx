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

//-----------------------------------------------------------------------------
//
//  class  DeepTile
//
//-----------------------------------------------------------------------------

#include "DcxDeepPixel.h"
#include "DcxChannelAlias.h"
#include "DcxImageFormat.h"

#include <OpenEXR/ImathBox.h>

#include <assert.h>

//----------------------------------------------------------------------------------------------------
// TODO: this seems missing from IlmBase...where to put this...?
template <class T>
std::ostream&
operator << (std::ostream& os,
             const IMATH_NAMESPACE::Box<T>& b)
{
    os << "[" << b.min.x << " " << b.min.y << " " << b.max.x << " " << b.max.y << "]";
    return os;
}
//----------------------------------------------------------------------------------------------------


// Uncomment this to get debug info:
//#define DCX_DEBUG_DEEPTILE 1

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//-----------------------------------------------------------------------------
//
// class DeepTile
//
//      Abstract class to manage a rectangular section of deep pixels.
//
//      This class is intended for simplifing common pixel-region loops that
//      step from pixel to pixel processing multiple channels simultaneously,
//      vs. using the IlmImfUtil deep classes which seem organized more for
//      per-plane processing and more targeted for deep texture use.
//
//      See the DeepImageTile classes for an example that wraps the IlmImfUtil
//      DeepImage class.
//
//-----------------------------------------------------------------------------

class DCX_EXPORT DeepTile
{
  public:

    //----------------------------------------------------------------------
    //
    // Hints at how the tile should be spatially accessed during writing of
    // deep pixels.
    // Some memory arrangements allow only sequential writing of the entire
    // tile buffer in order while others want it restricted per-scanline.
    //
    //----------------------------------------------------------------------

    enum WriteAccessMode
    {
        WRITE_DISABLED,         // Cannot write to tile
        WRITE_RANDOM,           // Any x,y pixel can be written to randomly
        WRITE_RANDOM_SCANLINE,  // Scanlines can be written randomly but pixels must be written in order (x argument ignored)
        WRITE_SEQUENTIAL        // Entire tile must be written in order (x & y arguments ignored)
    };


  public:

    //
    // Must provide a ChannelContext at a minimum.
    //

    DeepTile (ChannelContext& channel_ctx,
              WriteAccessMode write_access_mode=WRITE_DISABLED,
              bool tileYup=true);

    //
    // Assigns resolution and channel set.
    // The set of ChannelAliases is used to construct a map of ChannelIdx's to
    // ChannelAliases.  If multiple input ChannelAliases have the same ChannelIdx
    // destination only the first one is accepted and the others are ignored.
    //

    DeepTile (const IMATH_NAMESPACE::Box2i& display_window,
              const IMATH_NAMESPACE::Box2i& data_window,
              bool sourceWindowsYup,
              const ChannelAliasSet& channels,
              ChannelContext&  channel_ctx,
              WriteAccessMode write_access_mode=WRITE_DISABLED,
              bool tileYup=true);

    virtual ~DeepTile ();


    //
    // Are the data/display windows Y-up?  If true pixel access
    // method Y-coordinates are interpreted as Y-up.
    //

    bool    tileYup() const;


    const IMATH_NAMESPACE::Box2i&   dataWindow () const;
    int     x () const;
    int     y () const;
    int     r () const;
    int     t () const;
    int     w () const;
    int     h () const;


    const IMATH_NAMESPACE::Box2i&   displayWindow () const;
    int     fx () const;
    int     fy () const;
    int     fr () const;
    int     ft () const;
    int     fw () const;
    int     fh () const;


    //
    // ChannelSet for tile, derived from channelAliasSet.
    //

    const Dcx::ChannelSet&  channels () const;


    //
    // Number of color/aov channels in each DeepPixel.
    //

    size_t  numChannels () const;


    //
    // Map of ChannelIdx->ChannelAliases assigned to this tile.
    //

    const ChannelIdxToAliasMap& channelAliasMap () const;


    //
    // Return a ChannelAlias pointer corresponding to a ChannelIdx
    //

    const ChannelAlias* getChannelAlias(ChannelIdx) const;


    //
    // Hints at how the tile should be spatially accessed during writing of
    // deep pixels.
    //

    WriteAccessMode writeAccessMode () const;


    bool    writable() const;
    bool    hasSpMasks () const;
    bool    hasFlags () const;


    //
    // Returns true if pixel x,y is inside data window.
    //

    bool isActivePixel (int x, int y) const;


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
                               Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                               , bool debug=false
#endif
                               ) const=0;

    virtual bool getSampleMetadata (int x,
                                    int y,
                                    size_t sample,
                                    Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                    , bool debug=false
#endif
                                    ) const=0;


    //
    // Writes a DeepPixel to a pixel-space location (x, y) in the deep channels.
    // If xy is out of bounds or the tile can't be written to, the deep pixel
    // is left empty and false is returned.
    // x or y arguments might be ignored depending on write access mode.
    //
    // Default implementation always returns false.
    //

    virtual bool setDeepPixel (int x,
                               int y,
                               const Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                               , bool debug=false
#endif
                               );


    //
    // Writes an empty DeepPixel (0 samples) at a pixel-space location (x, y).
    // Returns false if x,y is out of bounds or the tile can't be written to.
    // x or y arguments might be ignored depending on write access mode.
    //
    // Default implementation always returns false.
    //

    virtual bool clearDeepPixel (int x,
                                 int y
#ifdef DCX_DEBUG_DEEPTILE
                                 , bool debug=false
#endif
                                 );

  protected:
    //
    // Copy constructor only for subclasses
    //
    DeepTile (const DeepTile&);

    //
    // Build the active ChannelSet from a set of ChannelAliases.
    // Also updates the ChannelAliasMap.
    // If spmask or flag channels are in the set this will update the
    // spmask channel count and the flags channel.
    //
    virtual void updateChannels (const ChannelAliasSet& channels);

    // Assigned vars:
    WriteAccessMode         m_write_access_mode;    // Supported spatial write-access mode
    bool                    m_tile_yUp;             // Is Y-axis of tile pointing up(industry-std) or down(exr-std)?
    IMATH_NAMESPACE::Box2i  m_display_window;       // Bbox of target format (normally the displayWindow)
    IMATH_NAMESPACE::Box2i  m_data_window;          // Bbox of active pixel area, *** possibly flipped in Y ***!
    ChannelContext*         m_channel_ctx;          // Context for channels

    // Derived vars:
    Dcx::ChannelSet         m_channels;             // ChannelSet shared by all pixels in tile
    ChannelIdxToAliasMap    m_channel_aliases;      // Map of ChannelIdx->ChannelAliases
    size_t                  m_num_spmask_chans;     // Number of active SpMask channels (1-8) - TODO: deprecate!
    Dcx::ChannelIdx         m_flags_channel;        // Flags channel

};



//-----------------
// Inline Functions
//-----------------

inline bool DeepTile::tileYup () const { return m_tile_yUp; }
inline const IMATH_NAMESPACE::Box2i& DeepTile::displayWindow () const { return m_display_window; }
inline const IMATH_NAMESPACE::Box2i& DeepTile::dataWindow () const { return m_data_window; }
inline const Dcx::ChannelSet& DeepTile::channels () const { return m_channels; }
inline const ChannelIdxToAliasMap& DeepTile::channelAliasMap () const { return m_channel_aliases; }
inline const ChannelAlias* DeepTile::getChannelAlias(ChannelIdx z) const
{
    ChannelIdxToAliasMap::const_iterator it = m_channel_aliases.find(z);
    if (it == m_channel_aliases.end())
        return NULL;
    return it->second;
}
inline DeepTile::WriteAccessMode DeepTile::writeAccessMode () const { return m_write_access_mode; }
inline bool DeepTile::writable () const { return (m_write_access_mode > WRITE_DISABLED); }
inline
bool DeepTile::isActivePixel (int x, int y) const { return !(x < m_data_window.min.x || y < m_data_window.min.y ||
                                                             x > m_data_window.max.x || y > m_data_window.max.y); }
inline size_t DeepTile::numChannels () const { return m_channels.size(); }
inline int DeepTile::x () const { return m_data_window.min.x; }
inline int DeepTile::y () const { return m_data_window.min.y; }
inline int DeepTile::r () const { return m_data_window.max.x; }
inline int DeepTile::t () const { return m_data_window.max.y; }
inline int DeepTile::w () const { return (m_data_window.max.x - m_data_window.min.x + 1); }
inline int DeepTile::h () const { return (m_data_window.max.y - m_data_window.min.y + 1); }
inline int DeepTile::fx () const { return m_display_window.min.x; }
inline int DeepTile::fy () const { return m_display_window.min.y; }
inline int DeepTile::fr () const { return m_display_window.max.x; }
inline int DeepTile::ft () const { return m_display_window.max.y; }
inline int DeepTile::fw () const { return (m_display_window.max.x - m_display_window.min.x + 1); }
inline int DeepTile::fh () const { return (m_display_window.max.y - m_display_window.min.y + 1); }
inline
bool DeepTile::hasSpMasks () const { return (m_num_spmask_chans > 0); }
inline
bool DeepTile::hasFlags () const { return (m_flags_channel != Dcx::Chan_Invalid); }
inline
bool DeepTile::setDeepPixel (int, int, const Dcx::DeepPixel&
#ifdef DCX_DEBUG_DEEPTILE
                             , bool
#endif
                             ) { return false; }
inline
bool DeepTile::clearDeepPixel (int, int
#ifdef DCX_DEBUG_DEEPTILE
                               , bool
#endif
                               ) { return false; }


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DEEPTILE_H
