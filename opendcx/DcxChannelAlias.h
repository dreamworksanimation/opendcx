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
/// @file DcxChannelAlias.h

#ifndef INCLUDED_DCX_CHANNELALIAS_H
#define INCLUDED_DCX_CHANNELALIAS_H

//-----------------------------------------------------------------------------
//
//  class    ChannelAlias
//  typedef  ChannelAliasList
//  typedef  ChannelAliasSet
//  typedef  ChannelAliasMap
//  class    ChannelContext
//
//-----------------------------------------------------------------------------

#include "DcxChannelSet.h"

#include <OpenEXR/ImfPixelType.h>

#include <vector>
#include <map>
#include <iostream>
#include <assert.h>

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

// This should be in IlmImf somewhere...
DCX_EXPORT
const char*
pixelTypeString(OPENEXR_IMF_NAMESPACE::PixelType type);


//
// Split the name into separate layer & chan strings, if possible,
// returning true if successful.
// This splits at the last period ('.') in the string. If the
// name contains more than one period then those become part of the
// layer string.
//

DCX_EXPORT
void    splitName (const char* name,
                   std::string& layer,
                   std::string& chan);


//
// Returns the best matching standard ChannelIdx and layer name for
// the channel name (with no layer prefix,) or false if no match.
// If there's a match it also sets the channel's default name and
// PixelType for file I/O.
//

DCX_EXPORT
bool    matchStandardChannel (const char* channel_name,
                              std::string&    std_layer_name,
                              std::string&    std_chan_name,
                              ChannelIdx&     std_channel,
                              std::string&    std_io_name,
                              Imf::PixelType& std_io_type);



//-----------------------------------------------------------------------------
//
//  class ChannelAlias
//
//      There should be one of these for every available ChannelIdx that can be
//      accessed by a ChannelSet.  There can be ChannelAliases in multiple
//      ChannelLayers that point to the same ChannelIdx, but there's always a
//      single global list of ChannelIdx's.
//
//      This allows channels from multiple Parts & files to have different
//      names (aliases) but still align for compositing operations.
//      Ex.'rgb' and 'rgba' layers both refer to the same Chan_R/Chan_G/Chan_B
//      ChannelIdx's, but 'rgba' also includes Chan_A.
// 
//      Nested layers are informally supported by always considering the last
//      period '.' in the channel name to be the layer/channel separator, so
//      the layer name can have multiple separators describing the nesting.
//      ex. beauty.diffuse.R
//
//-----------------------------------------------------------------------------

class DCX_EXPORT ChannelAlias
{
    friend class ChannelContext;

  public:

    //----------------------------------------------------------------------------
    // Construct a layer/channel alias.
    //      name     user-facing name, not including the layer ('R', 'G', 'red',
    //              'green', 'alpha', 'Z', etc)
    //      layer    user-facing layer name ('rgba', 'beauty', 'beauty.diffuse')
    //      channel  global ChannelIdx
    //      io_name  name of channel for file I/O ('R' or 'AR'  vs. 'rgba.red' or
    //               'opacity.R')
    //      io_type  PixelType to use for file I/O
    //      kind     the 'kind' of channel this is, usually one of the predefined
    //               channels used to determine the channel's logical order inside
    //               the layer for sorting purposes.
    //               ex. Chan_R, Chan_G, Chan_UvS, Chan_UvT
    //----------------------------------------------------------------------------

    ChannelAlias (const char* name,
                  const char* layer,
                  ChannelIdx channel,
                  const char* io_name,
                  Imf::PixelType io_type,
                  ChannelIdx kind = Chan_Invalid);

    virtual ~ChannelAlias();

    //---------------------------------------------
    // The absolute ChannelIdx the alias points to.
    //---------------------------------------------

    ChannelIdx      channel () const;


    //---------------------------------------------------------------
    // Get the channel, layer, or concatenated <layer>.<channel> name
    //---------------------------------------------------------------

    const std::string&  name () const;
    const std::string&  layer () const;
    std::string         fullName () const;


    //--------------------------------------------------
    // Default name used in for file I/O
    //   ex. 'R'  vs. 'rgba.red'
    //
    // If this channel is not one of the predefined ones
    // this will be the same as fullName()
    //--------------------------------------------------

    std::string     fileIOName () const;


    //-------------------------------------------------------
    // The index position of the channel inside the layer.
    // If the channel kind is a predefined one then the index
    // order will be in the predefined sort order rather than
    // alphabetically by name, allowing channels to be
    // grouped logically by their function.
    //
    //      ex. R,G,B,A  vs. A,B,G,R
    //          S,T,P,Q  vs. P,Q,S,T
    //-------------------------------------------------------

    int  positionInLayer () const;


    //-------------------------------------------------------------
    // Default pixel data type to use when reading/writing to files
    //-------------------------------------------------------------

    Imf::PixelType  fileIOPixelType () const;


    //-------------------------------------------
    // The 'kind' of channel, from the predefined
    // list (Chan_R,Chan_G,Chan_Z, etc,) used to
    // sort the alias list in the layer.
    //-------------------------------------------

    ChannelIdx  kind () const;


    //----------------------------------------------------
    // Equality operator compares ChannelIdx.
    //----------------------------------------------------

    bool operator == (const ChannelAlias&) const;


    //-----------------------------------------
    // Used by the sort routine, keys on 'kind'
    //-----------------------------------------

    bool operator < (const ChannelAlias& b) const;


    //---------------------------------------
    // Output the name of the channel to the stream.
    //---------------------------------------

    friend std::ostream& operator << (std::ostream&,
                                      const ChannelSet&);


  protected:

    std::string     m_name;             // User-facing name of channel ('red', 'green', etc)
    std::string     m_layer;            // User-facing name of layer ('rgba', 'beauty', 'beauty.diffuse', etc)
    //
    ChannelIdx      m_channel;          // Absolute ChannelIdx this alias maps to (Chan_R, Chan_ZB, 55, 654, 1012)
    ChannelIdx      m_kind;             // If one of the predefined channel types (Chan_R, Chan_G, Chan_Z, Chan_DeepFlags)
    int             m_position;         // Sorted position index within layer (i.e. 0,1,2) derived from m_kind
    //
    std::string     m_io_name;          // Name of channel for file IO ('R' or 'AR'  vs. 'rgba.red' or 'opacity.R')
    Imf::PixelType  m_io_type;          // Channel I/O data type


    //------------------------------------------
    // Disabled copy constructor / operator
    //------------------------------------------

    ChannelAlias (const ChannelAlias&);
    ChannelAlias& operator = (const ChannelAlias&);

};


typedef  std::vector<ChannelAlias*>             ChannelAliasList;
typedef  std::set<ChannelAlias*>                ChannelAliasSet;
typedef  std::map<ChannelIdx, ChannelAlias*>    ChannelIdxToAliasMap;





//---------------------------------------------------------------------------
//
//
//  class ChannelContext
//
//      Context structure storing the global channel assignments and
//      maps for quick access to/from ChannelAliases and ChannelIdxs
//
//
//---------------------------------------------------------------------------

class DCX_EXPORT ChannelContext
{
  public:

    struct ChanOrder
    {
        ChannelIdx  channel;    // ChannelIdx
        uint32_t    order;      // Order in layer
    };

    struct Layer
    {
        std::string             name;
        std::vector<ChanOrder>  channels;
    };


    typedef std::map<std::string, int> AliasNameToListMap;
    typedef std::map<ChannelIdx,  int> ChannelIdxToListMap;
    typedef std::map<std::string, int> LayerNameToListMap;


  public:

    ChannelContext();

    virtual ~ChannelContext();

    //---------------------------------------------------------------
    //
    // Returns the last assigned ChannelIdx.
    //
    // This value can be used to size channel arrays as it represents
    // the current maximum channel count for this context.
    // Note that this value does not represent *active* channels,
    // only channel definitions, whereas a ChannelSet is used to
    // define a set of active channels.
    //
    // If no arbitrary channels have been added to this context the
    // value will be Chan_ArbitraryStart-1.
    //
    //---------------------------------------------------------------

    ChannelIdx  lastAssignedChannel () const;


    //---------------------------------------------------------------------
    //
    // Get and/or create a channel, returning a ChannelIdx or ChannelAlias.
    // Returns Chan_Invalid or NULL if there was an error creating it.
    //
    // Note that multiple channel names can map to the same ChannelIdx but
    // each name will have a unique ChannelAlias.
    // ex. 'R', 'rgba.R', 'rgb.red' will all be mapped to Chan_R with each
    // getting a unique ChannelAlias.
    //
    // Unrecognized channel names like 'mylayer.foo' and 'mylayer.F' will
    // each be assigned unique ChannelIdxs unless the duplicate
    // ChannelAliases were added using addChannelAlias() with the same
    // ChannelIdx from the first created alias.
    //
    //---------------------------------------------------------------------

    ChannelIdx  getChannel (const char* name);
    ChannelIdx  getChannel (const std::string& name);

    ChannelAlias*  getChannelAlias (const char* name);
    ChannelAlias*  getChannelAlias (const std::string& name);


    //---------------------------------------------------------------------
    //
    // Get channel or layer.channel name from a ChannelIdx.
    // Returns 'unknown' if the ChannelIdx doesn't exist (is out of range.)
    //
    //---------------------------------------------------------------------

    const char*     getChannelName (ChannelIdx channel) const;
    std::string     getChannelFullName (ChannelIdx channel) const;


    //----------------------------------------------------------------------
    //
    // Find channel by name or ChannelIdx and return a ChannelAlias pointer,
    // or NULL if not found.
    //
    //----------------------------------------------------------------------

    ChannelAlias*   findChannelAlias (const char* name) const;
    ChannelAlias*   findChannelAlias (const std::string& name) const;
    ChannelAlias*   findChannelAlias (ChannelIdx channel) const;


    //--------------------------------------------------------------------------------
    //
    // Add a new ChannelAlias to the context, either by passing in a pre-allocated
    // ChannelAlias or having the context construct it.
    // In both cases the ChannelContext takes ownership of the pointer and
    // deletes the pointers in the destructor.
    //
    // If the assigned channel is Chan_Invalid then no specfic channel slot is
    // being requested so the next available ChannelIdx is assigned, incrementing
    // lastAssignedChannel().
    //
    //  chan_name   user-facing name, not including the layer ('R', 'G', 'red',
    //              'green', 'alpha', 'Z', 'ZBack', etc)
    //  layer_name  user-facing layer name ('rgba', 'beauty', 'beauty.diffuse')
    //  channel     absolute ChannelIdx - if Chan_Invalid a new ChannelIdx is assinged
    //  io_name     name to use for exr file I/O ('R' or 'AR'  vs. 'rgba.red' or
    //              'opacity.R')
    //  io_type     PixelType to use for file I/O
    //  kind        the 'kind' of channel this is, usually one of the predefined
    //              channels used to determine the channel's logical order inside
    //              a layer for sorting purposes.
    //              ex. Chan_R, Chan_G, Chan_UvS, Chan_UvT
    //--------------------------------------------------------------------------------

    ChannelAlias*   addChannelAlias (ChannelAlias* alias);

    ChannelAlias*   addChannelAlias (const std::string& chan_name,
                                     const std::string& layer_name,
                                     ChannelIdx         channel,
                                     const std::string& io_name,
                                     Imf::PixelType     io_type,
                                     ChannelIdx         kind);


    //---------------------------------------------
    //
    // Read-only access to the shared lists & maps.
    //
    //---------------------------------------------

    const ChannelAliasList&     channelAliasList () const;
    const AliasNameToListMap&   channelNameToAliasListMap () const;
    const ChannelIdxToListMap&  channelaliasToChannelMap () const;


    //---------------------------------------------------------------
    //
    // Print channel or '<layer>.<channel>' name to an output stream.
    //
    //---------------------------------------------------------------

    void    printChannelName (std::ostream&, const ChannelIdx&) const;
    void    printChannelFullName (std::ostream&, const ChannelIdx&) const;


  protected:

    ChannelIdx              m_last_assigned;                // Most recently assigned custom channel
    //
    ChannelAliasList        m_channelalias_list;            // List of all ChannelAliases bound to layers
    AliasNameToListMap      m_channelalias_name_map;        // Map of channel names -> m_channelalias_list index
    ChannelIdxToListMap     m_channelalias_channel_map;     // Map of ChannelIdxs -> m_channelalias_list index
    //
    std::vector<Layer>      m_layers;                       // List of Layers
    LayerNameToListMap      m_layer_name_map;               // Map of layer names -> m_layers index

};



//-----------------
// Inline Functions
//-----------------

inline const std::string& ChannelAlias::name () const { return m_name; }
inline const std::string& ChannelAlias::layer () const { return m_layer; }
inline ChannelIdx ChannelAlias::channel () const { return m_channel; }
inline int ChannelAlias::positionInLayer () const { return m_position; }
inline Imf::PixelType ChannelAlias::fileIOPixelType () const { return m_io_type; }
inline ChannelIdx ChannelAlias::kind () const { return m_kind; }
inline bool ChannelAlias::operator < (const ChannelAlias& b) const { return (m_kind < b.m_kind); }
//-----------------
inline ChannelAlias* ChannelContext::getChannelAlias (const std::string& name) { return getChannelAlias(name.c_str()); }
inline ChannelIdx ChannelContext::getChannel (const char* name)
{
    ChannelAlias* chan = getChannelAlias(name);
    if (chan)
        return chan->channel();
    return Chan_Invalid;
}
inline ChannelIdx ChannelContext::getChannel (const std::string& name) { return getChannel(name.c_str()); }
inline ChannelIdx ChannelContext::lastAssignedChannel () const { return m_last_assigned; }
//
inline const ChannelAliasList&
ChannelContext::channelAliasList () const { return m_channelalias_list; }
inline const ChannelContext::AliasNameToListMap&
ChannelContext::channelNameToAliasListMap () const { return m_channelalias_name_map; }
inline const ChannelContext::ChannelIdxToListMap&
ChannelContext::channelaliasToChannelMap () const { return m_channelalias_channel_map; }
//
inline void
ChannelContext::printChannelName (std::ostream& os, const ChannelIdx& channel) const { os << getChannelName(channel); }
inline void
ChannelContext::printChannelFullName (std::ostream& os, const ChannelIdx& channel) const { os << getChannelFullName(channel); }


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_CHANNELALIAS_H
