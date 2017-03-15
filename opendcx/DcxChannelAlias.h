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

//=============================================================================
//
//  class    ChannelAlias
//
//  typedef  ChannelAliasPtrList
//  typedef  ChannelAliasPtrSet
//  typedef  ChannelIdxToAliasMap
//
//=============================================================================

#include "DcxChannelSet.h"

#include <vector>
#include <map>
#include <iostream>


//-------------------------
//!rst:cpp:begin::
//.. _channelalias_class:
//
//ChannelAlias
//============
//-------------------------


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//============================
//
//  class ChannelAlias
// 
//============================
//-----------------------------------------------------------------------------
//
//  :ref:`channelcontext_class` stores a list of these structures to map
//  channel names to channel indices and vice-versa.
//
//  There should be one of these for every available ChannelIdx that can be
//  accessed by a ChannelSet.  There can be ChannelAliases in multiple
//  ChannelLayers that point to the same ChannelIdx, but there's always a
//  single global list of ChannelIdx's.
//
//  This allows channels from multiple Parts & files to have different
//  names (aliases) but still align for compositing operations.
//  Ex.'rgb' and 'rgba' layers both refer to the same Chan_R/Chan_G/Chan_B
//  ChannelIdx's, but 'rgba' also includes Chan_A.
//
//  Nested layers are informally supported by always considering the last
//  period '.' in the channel name to be the layer/channel separator, so
//  the layer name can have multiple separators describing the nesting.
//  ex. beauty.diffuse.R
//
//-----------------------------------------------------------------------------

class DCX_EXPORT ChannelAlias
{
    friend class ChannelContext;

  public:

    //-----------------------------------------------------------------------------
    //
    //  Construct a layer/channel alias.
    //
    //  * name      - user-facing name, not including the layer ('R', 'G', 'red',
    //                green', 'alpha', 'Z', etc)
    //  * layer     - user-facing layer name ('rgba', 'beauty', 'beauty.diffuse')
    //  * channel   - global ChannelIdx
    //  * position  - sorted position index within layer (i.e. 0,1,2)
    //  * io_name   - name of channel for file I/O ('R' or 'AR'  vs. 'rgba.red'
    //                or 'opacity.R')
    //  * io_type  - PixelType to use for file I/O
    //  * io_part  - Part index the channel writes to (TODO: finish support of this)
    //
    //-----------------------------------------------------------------------------

    ChannelAlias (const char*                       name,
                  const char*                       layer,
                  ChannelIdx                        channel,
                  uint32_t                          position,
                  const char*                       io_name,
                  OPENEXR_IMF_NAMESPACE::PixelType  io_type,
                  int                               io_part=0);

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


    //----------------------------------------------------
    // The index position of the channel inside the layer,
    // allowing channels to be grouped logically by their
    // function.
    //
    //      ex. R,G,B,A  vs. A,B,G,R
    //          S,T,P,Q  vs. P,Q,S,T
    //----------------------------------------------------

    int     layerPosition () const;


    //-------------------------------------------------------------
    // Default pixel data type to use when reading/writing to files
    //-------------------------------------------------------------

    OPENEXR_IMF_NAMESPACE::PixelType    fileIOPixelType () const;


    //-------------------------------------------------------------
    // Default Part index to use when reading/writing to files
    //-------------------------------------------------------------

    int     fileIOPartIndex () const;


    //----------------------------------------------------
    // Equality operator compares ChannelIdx.
    //----------------------------------------------------

    bool operator == (const ChannelAlias&) const;


    //-----------------------------------------------
    // Used by the sort routine, compares on position
    //-----------------------------------------------

    bool operator < (const ChannelAlias& b) const;


    //--------------------------------------------------
    // Outputs the full name of the alias to the stream.
    //--------------------------------------------------

    friend std::ostream& operator << (std::ostream&,
                                      const ChannelAlias&);


  protected:

    std::string     m_name;             // User-facing name of channel ('red', 'green', etc)
    std::string     m_layer;            // User-facing name of layer ('rgba', 'beauty', 'beauty.diffuse', etc)
    //
    ChannelIdx      m_channel;          // Absolute ChannelIdx this alias maps to (Chan_R, Chan_ZBack, 55, 654, 1012)
    int             m_position;         // Sorted position index within layer (i.e. 0,1,2)
    //
    std::string     m_io_name;          // Name of channel for file IO ('R' or 'AR'  vs. 'rgba.red' or 'opacity.R')
    OPENEXR_IMF_NAMESPACE::PixelType  m_io_type;  // Channel I/O data type
    int             m_io_part;          // Part index the channel writes to (TODO: finish support of this)


    //------------------------------------------
    // Disabled copy constructor / operator
    //------------------------------------------

    ChannelAlias (const ChannelAlias&);
    ChannelAlias& operator = (const ChannelAlias&);

};

//=============================================================================

typedef  std::vector<ChannelAlias*>             ChannelAliasPtrList;
typedef  std::set<ChannelAlias*>                ChannelAliasPtrSet;
typedef  std::map<ChannelIdx, ChannelAlias*>    ChannelIdxToAliasMap;

//=============================================================================

//--------------
//!rst:cpp:end::
//--------------

//-----------------
// Inline Functions
//-----------------

inline const std::string& ChannelAlias::name () const { return m_name; }
inline const std::string& ChannelAlias::layer () const { return m_layer; }
inline ChannelIdx ChannelAlias::channel () const { return m_channel; }
inline int ChannelAlias::layerPosition () const { return m_position; }
inline OPENEXR_IMF_NAMESPACE::PixelType ChannelAlias::fileIOPixelType () const { return m_io_type; }
inline int ChannelAlias::fileIOPartIndex () const { return m_io_part; }
inline bool ChannelAlias::operator < (const ChannelAlias& b) const { return (m_position < b.m_position); }


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_CHANNELALIAS_H
