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
/// @file DcxDeepTile.cpp


#include "DcxDeepTile.h"


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


DeepTile::DeepTile (ChannelContext& channel_ctx,
                    WriteAccessMode write_access_mode,
                    bool tileYup) :
    m_write_access_mode(write_access_mode),
    m_tile_yUp(tileYup),
    m_display_window(IMATH_NAMESPACE::V2i(0,0), IMATH_NAMESPACE::V2i(0,0)),
    m_data_window(IMATH_NAMESPACE::V2i(0,0), IMATH_NAMESPACE::V2i(0,0)),
    m_channel_ctx(&channel_ctx),
    //
    m_num_spmask_chans(0),
    m_flags_channel(Dcx::Chan_Invalid)
{
    //
}

DeepTile::DeepTile (const DeepTile& b) :
    m_write_access_mode(b.m_write_access_mode),
    m_tile_yUp(b.m_tile_yUp),
    m_display_window(b.m_display_window),
    m_data_window(b.m_data_window),
    m_channel_ctx(b.m_channel_ctx),
    //
    m_channels(b.m_channels),
    m_channel_aliases(b.m_channel_aliases),
    m_num_spmask_chans(b.m_num_spmask_chans),
    m_flags_channel(b.m_flags_channel)
{
    //
}

DeepTile::DeepTile (const IMATH_NAMESPACE::Box2i& display_window,
                    const IMATH_NAMESPACE::Box2i& data_window,
                    bool sourceWindowsYup,
                    const ChannelAliasPtrSet& channels,
                    ChannelContext& channel_ctx,
                    WriteAccessMode write_access_mode,
                    bool tileYup) :
    m_write_access_mode(write_access_mode),
    m_tile_yUp(tileYup),
    m_display_window(display_window),
    m_data_window(data_window),
    m_channel_ctx(&channel_ctx),
    //
    m_flags_channel(Dcx::Chan_Invalid)
{
    // Flip data window vertically if source windows are flipped:
    if (tileYup != sourceWindowsYup)
    {
        const int ot = m_data_window.max.y;
        m_data_window.max.y = m_display_window.max.y - m_data_window.min.y;
        m_data_window.min.y = m_display_window.max.y - ot;
    }
    updateChannels(channels);
}


/*virtual*/
void
DeepTile::updateChannels (const ChannelAliasPtrSet& channels)
{
    m_channels.clear();
    m_channel_aliases.clear();
    m_num_spmask_chans = 0;
    m_flags_channel = Dcx::Chan_Invalid;
    for (ChannelAliasPtrSet::const_iterator it=channels.begin(); it != channels.end(); ++it)
    {
        ChannelAlias* c = *it;
        if (!c || c->channel() == Chan_Invalid || m_channels.contains(c->channel()))
            continue; // don't crash and ignore duplicates

        m_channels += c->channel();
        m_channel_aliases[c->channel()] = c;

        if (c->channel() >= Chan_SpBits1 && c->channel() <= Chan_SpBitsLast)
            ++m_num_spmask_chans;
        if (c->channel() == Chan_DeepFlags && m_flags_channel == Chan_Invalid)
            m_flags_channel = Chan_DeepFlags;
    }
}


/*virtual*/
DeepTile::~DeepTile ()
{
   //
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
