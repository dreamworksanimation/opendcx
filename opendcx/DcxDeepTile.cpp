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
                    bool yAxisUp) :
    PixelTile(channel_ctx,
              yAxisUp),
    m_write_access_mode(write_access_mode),
    m_flags_channel(Dcx::Chan_Invalid)
{
    m_spmask_channel[0] = m_spmask_channel[1] = Dcx::Chan_Invalid;
}

DeepTile::DeepTile (const Imf::Header& header,
                    ChannelContext& channel_ctx,
                    WriteAccessMode write_access_mode,
                    bool yAxisUp) :
    PixelTile(header,
              channel_ctx,
              yAxisUp),
    m_write_access_mode(write_access_mode),
    m_flags_channel(Dcx::Chan_Invalid)
{
    m_spmask_channel[0] = m_spmask_channel[1] = Dcx::Chan_Invalid;
}

DeepTile::DeepTile (const IMATH_NAMESPACE::Box2i& display_window,
                    const IMATH_NAMESPACE::Box2i& data_window,
                    bool sourceWindowsYup,
                    const ChannelSet& channels,
                    ChannelContext& channel_ctx,
                    WriteAccessMode write_access_mode,
                    bool yAxisUp) :
    PixelTile(display_window,
              data_window,
              sourceWindowsYup,
              channels,
              channel_ctx,
              yAxisUp),
    m_write_access_mode(write_access_mode),
    m_flags_channel(Dcx::Chan_Invalid)
{
    m_spmask_channel[0] = m_spmask_channel[1] = Dcx::Chan_Invalid;
}

DeepTile::DeepTile (const IMATH_NAMESPACE::Box2i& data_window,
                    int top_reference,
                    bool sourceWindowsYup,
                    const ChannelSet& channels,
                    ChannelContext& channel_ctx,
                    WriteAccessMode write_access_mode,
                    bool yAxisUp) :
    PixelTile(data_window,
              top_reference,
              sourceWindowsYup,
              channels,
              channel_ctx,
              yAxisUp),
    m_write_access_mode(write_access_mode),
    m_flags_channel(Dcx::Chan_Invalid)
{
    m_spmask_channel[0] = m_spmask_channel[1] = Dcx::Chan_Invalid;
}

DeepTile::DeepTile (const DeepTile& b) :
    PixelTile(b),
    m_write_access_mode(b.m_write_access_mode),
    m_flags_channel(b.m_flags_channel)
{
    m_spmask_channel[0] = b.m_spmask_channel[0];
    m_spmask_channel[1] = b.m_spmask_channel[1];
}


/*virtual*/
void
DeepTile::setChannels (const ChannelSet& channels,
                       bool force)
{
    PixelTile::setChannels(channels, force);
    // Get the spmask and flag channel assignments:
    m_spmask_channel[0] = m_spmask_channel[1] = Dcx::Chan_Invalid;
    m_flags_channel = Dcx::Chan_Invalid;
    foreach_channel(z, channels)
    {
        if      (z == Chan_SpBits1 && m_spmask_channel[0] == Dcx::Chan_Invalid)
            m_spmask_channel[0] = Chan_SpBits1;
        else if (z == Chan_SpBits2 && m_spmask_channel[1] == Dcx::Chan_Invalid)
            m_spmask_channel[1] = Chan_SpBits2;
        else if (z == Chan_DeepFlags && m_flags_channel == Chan_Invalid)
            m_flags_channel = Chan_DeepFlags;
    }
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
