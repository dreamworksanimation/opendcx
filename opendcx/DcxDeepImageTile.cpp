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
/// @file DcxDeepImageTile.cpp


#include "DcxDeepImageTile.h"

#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfDeepImage.h>

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


DeepImageInputTile::DeepImageInputTile (ChannelContext& channel_ctx,
                                        bool tileYup) :
    DeepTile(channel_ctx, WRITE_DISABLED, tileYup),
    m_image_level(NULL)
{
    //
}


DeepImageInputTile::DeepImageInputTile (const Imf::DeepImage& image,
                                        ChannelContext& channel_ctx,
                                        bool tileYup) :
    DeepTile(channel_ctx, WRITE_DISABLED, tileYup),
    m_image_level(NULL)
{
    if (image.numLevels() > 0)
    {
        copyFromLevel(image, 0/*level*/);
        m_display_window = m_data_window;
    }
    else
    {
        // TODO: Level out of range - throw an exception?
    }
}


DeepImageInputTile::DeepImageInputTile (const Imf::Header& header,
                                        const Imf::DeepImage& image,
                                        ChannelContext& channel_ctx,
                                        bool tileYup) :
    DeepTile(channel_ctx, WRITE_DISABLED, tileYup),
    m_image_level(NULL)
{
    if (image.numLevels() > 0)
    {
        m_display_window = header.displayWindow();
        copyFromLevel(image, 0/*level*/);
    }
    else
    {
        // TODO: Level out of range - throw an exception?
    }
}


bool
DeepImageInputTile::copyFromLevel (const Imf::DeepImage& image,
                                   int level)
{
    if (level >= image.numLevels())
        return false;

    m_image_level = &image.level(level);
    m_data_window = image.dataWindowForLevel(level);

    if (m_tile_yUp)
    {
        // Flip data window vertically:
        const int ot = m_data_window.max.y;
        m_data_window.max.y = m_display_window.max.y - m_data_window.min.y;
        m_data_window.min.y = m_display_window.max.y - ot;
    }
    m_channels.clear();
    m_channel_aliases.clear();
    m_num_spmask_chans = 0;
    m_flags_channel = Dcx::Chan_Invalid;

#if 0//def DCX_DEBUG_DEEPTILE
std::cout << "DeepImageInputTile::copyFromLevel(" << this << "): level=" << level << std::endl;
std::cout << "  display_window" << m_display_window << std::endl;
std::cout << "     data_window" << m_data_window << std::endl;
#endif
    if (!updateChannelPtrs())
        return false;

    return true;
}

bool
DeepImageInputTile::updateChannelPtrs ()
{
    if (!m_image_level)
        return false;

#ifdef DCX_DEBUG_DEEPTILE
std::cout << "DeepImageInputTile::updateChannelPtrs(" << this << ") m_image_level=" << m_image_level << std::endl;
#endif
    // Copy the channel data ptrs out of the DeepImageLevel:
    m_chan_ptrs.clear();
    std::vector<Dcx::ChannelIdx> chans;
    std::vector<const Imf::DeepImageChannel*> ptrs;
    Dcx::ChannelAliasSet tile_channels; // set of channels to initialize tile to

    assert(m_channel_ctx);

    for (Imf::DeepImageLevel::ConstIterator it=m_image_level->begin(); it != m_image_level->end(); ++it)
    {
#if 1
        ChannelAlias* c = m_channel_ctx->getChannelAlias(it.name());
        if (!c || c->channel() == Dcx::Chan_Invalid)
            continue; // error creating the alias!  TODO: throw exception?

        chans.push_back(c->channel());
        ptrs.push_back(&it.channel());

        tile_channels.insert(c);

#else
        Dcx::ChannelIdx z = Dcx::getChannel(it.name().c_str());

        if (z == Dcx::Chan_DeepFlags && m_flags_channel == Dcx::Chan_Invalid)
        {
            m_flags_channel = z;
            chans.push_back(z);
            ptrs.push_back(&it.channel());
        }
        else if (z >= Chan_SpBits1 && z <= Chan_SpBits2 &&
                 !spmask_channels.contains(z))
        {
            ++m_num_spmask_chans;
            spmask_channels += z;
            chans.push_back(z);
            ptrs.push_back(&it.channel());
        }
        else if (z != Dcx::Chan_Invalid)
        {
            // Translate channel "Z" into Chan_ZFront:
            if (z == Dcx::Chan_Z)
                z = Dcx::Chan_ZFront;

            m_channels += z;
            chans.push_back(z);
            ptrs.push_back(&it.channel());
        }
#endif
    }

    // Update the active channel set:
    DeepTile::updateChannels(tile_channels);

    m_chan_ptrs.resize(m_channel_ctx->lastAssignedChannel());
    memset(&(m_chan_ptrs[0]), 0, sizeof(Imf::DeepImageChannel*)*m_chan_ptrs.size()); // << TODO: is this needed?
    for (size_t i=0; i < chans.size(); ++i)
        m_chan_ptrs[chans[i]] = ptrs[i];

#ifdef DCX_DEBUG_DEEPTILE
std::cout << "    chan ptrs[";
for (size_t i=0; i < m_chan_ptrs.size(); ++i)
    if (m_chan_ptrs[i] != 0)
        std::cout << " " << i << "(" << m_chan_ptrs[i] << ")";
std::cout << "]" << std::endl;
#endif

    return true;
}


/*virtual*/
size_t
DeepImageInputTile::getNumSamplesAt (int x, int y) const
{
    if (!m_image_level || !isActivePixel(x, y))
        return 0;
    return m_image_level->sampleCounts()(x, (m_tile_yUp)?(m_display_window.max.y-y):y);
}


/*virtual*/
bool
DeepImageInputTile::getDeepPixel (int x,
                                  int y,
                                  Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                  , bool debug
#endif
                                  ) const
{
    pixel.clear();
    if (!m_image_level)
    {
#if 0//def DCX_DEBUG_DEEPTILE
        if (debug)
            std::cout << x << ":" << y << ":getDeepPixel() no active deep level " << m_data_window << std::endl;
#endif
        return false;
    }
    if (!isActivePixel(x, y))
    {
#if 0//def DCX_DEBUG_DEEPTILE
        if (debug)
            std::cout << x << ":" << y << ":getDeepPixel() outside data_window " << m_data_window << std::endl;
#endif
        return false;
    }

    if (m_channels.empty())
        return true;

    const size_t nSamples = m_image_level->sampleCounts()(x, (m_tile_yUp)?(m_display_window.max.y-y):y);
#ifdef DCX_DEBUG_DEEPTILE
    if (debug)
        std::cout << x << ":" << y << ":getDeepPixel() samples=" << nSamples << std::endl;
#endif
    if (nSamples == 0)
        return true;

    // Copy sample data out of channel ptrs:
    pixel.reserve(nSamples);

    Dcx::ChannelSet copy_channels(m_channels);
    copy_channels -= Dcx::Mask_Deep;
    copy_channels -= Dcx::Mask_DeepMetadata;
    copy_channels -= Dcx::Mask_Z;
    pixel.setChannels(copy_channels);

    const bool have_Zb  = (m_channels.contains(Dcx::Chan_ZBack));

    Dcx::DeepSegment ds;
    for (size_t sample=0; sample < nSamples; ++sample)
    {
        ds.Zf = ds.Zb = getChannelSampleValueAt(x, y, sample, m_chan_ptrs[Dcx::Chan_ZFront]);
        // Skip samples with negative, infinite or nan Zfront:
        if (ds.Zf < 0.0f || isinf(ds.Zf) || isnan(ds.Zf))
            continue;

        if (have_Zb)
        {
            ds.Zb = getChannelSampleValueAt(x, y, sample, m_chan_ptrs[Dcx::Chan_ZBack]);
            // Clamp Zback to reasonable values - allow infinity:
            if (isnan(ds.Zb) || ds.Zb < ds.Zf)
                ds.Zb = ds.Zf;
        }

        ds.index = (int)sample;

        // Extract metadata from input channels:
        if (!getSampleMetadata(x, y, sample, ds.metadata))
            return false;

#ifdef DCX_DEBUG_DEEPTILE
        if (debug) {
            std::cout << "  " << sample << " " << std::fixed << std::cout.precision(8) << ds.Zf << "-" << ds.Zb;
            std::cout << ", flags=("; ds.printFlags(std::cout); std::cout << ")";
            std::cout << " [";
        }
#endif
        // Add segment and copy pixel data:
        const size_t dsindex = pixel.append(ds);
        Dcx::Pixelf& p = pixel.getSegmentPixel(dsindex);
        foreach_channel(z, copy_channels)
        {
            p[*z] = getChannelSampleValueAt(x, y, sample, m_chan_ptrs[*z]);
#ifdef DCX_DEBUG_DEEPTILE
            if (debug)
                std::cout << " " << m_channel_ctx->getChannelName(*z) << "=" << std::fixed << std::cout.precision(6) << p[*z];
#endif
        }
#ifdef DCX_DEBUG_DEEPTILE
        if (debug)
        {
            std::cout << " ]" << std::endl;
            ds.spMask().printPattern(std::cout, "    ");
        }
#endif
    }
    return true;
}


/*virtual*/
bool
DeepImageInputTile::getSampleMetadata (int x,
                                       int y,
                                       size_t sample,
                                       Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                       , bool /*debug*/
#endif
                                       ) const
{
    if (m_num_spmask_chans == 2)
    {
        metadata.spmask.fromFloat(getChannelSampleValueAt(x, y, sample, m_chan_ptrs[Dcx::Chan_SpBits1]),
                                  getChannelSampleValueAt(x, y, sample, m_chan_ptrs[Dcx::Chan_SpBits2]));

    }
    else if (m_num_spmask_chans == 1)
    {
        //metadata.spmask = Dcx::spMask4From1Float(getChannelSampleValueAt(x, y, sample, m_chan_ptrs[Dcx::Chan_SpBits1]));

    }
    else
    {
        metadata.spmask = Dcx::SpMask8::zeroCoverage; // default to zero coverage (legacy data)
    }

    // Extract flags from flags channel (convert from floating-point integer value):
    if (m_flags_channel != Dcx::Chan_Invalid)
        metadata.flags = (Dcx::DeepFlag)floorf(getChannelSampleValueAt(x, y, sample, m_chan_ptrs[m_flags_channel]));
    else
        metadata.flags = Dcx::DEEP_EMPTY_FLAG;

    return true;
}


//-------------------------------------------------------------------------
//-------------------------------------------------------------------------


DeepImageOutputTile::DeepImageOutputTile (const IMATH_NAMESPACE::Box2i& display_window,
                                          const IMATH_NAMESPACE::Box2i& data_window,
                                          bool sourceWindowsYup,
                                          const ChannelAliasSet& channels,
                                          ChannelContext& channel_ctx,
                                          bool tileYup) :
    DeepTile (display_window, data_window, sourceWindowsYup, channels, channel_ctx, WRITE_RANDOM, tileYup),
    m_file(0)
{
#if 0
    // Make sure output channels have Z's, metadata enabled:
    m_channels += Dcx::Mask_Deep;
    switch (m_num_spmask_chans)
    {
    case 1: m_channels += Mask_SpMask4; break;
    default:
    case 2: m_channels += Mask_SpMask8; break;
    //case 8: m_channels += Mask_SpMask4; break;
    }
    m_channels += m_flags_channel;
#endif
    resizeDataWindow(data_window);
}


DeepImageOutputTile::DeepImageOutputTile (const DeepTile& b) :
    DeepTile(b),
    m_file(0)
{
    resizeDataWindow(m_data_window);
}


DeepImageOutputTile::~DeepImageOutputTile ()
{
    const size_t nLines = m_deep_lines.size();
    for (size_t y=0; y < nLines; ++y)
        delete m_deep_lines[y];
    delete m_file;
}


size_t
DeepImageOutputTile::bytesUsed () const
{
    size_t count = 0;
    size_t nLines = m_deep_lines.size();
    for (size_t j=0; j < nLines; ++j)
    {
        const DeepLine* dl = m_deep_lines[j];
        if (dl)
        {
            int chan_index = 0;
            foreach_channel(z, m_channels)
                count += dl->channel_arrays[chan_index].size();
        }
    }
    return (count * sizeof(float));
}


void
DeepImageOutputTile::deleteDeepLines ()
{
    const size_t nLines = m_deep_lines.size();
    for (size_t j=0; j < nLines; ++j)
    {
        delete m_deep_lines[j];
        m_deep_lines[j] = 0;
    }
}


void
DeepImageOutputTile::resizeDataWindow (const IMATH_NAMESPACE::Box2i& data_window)
{
    deleteDeepLines();
    m_data_window = data_window;
    size_t nLines = std::max(0, h());
    m_deep_lines.resize(nLines);
    memset(&m_deep_lines[0], 0, nLines*sizeof(DeepLine*));
}


/*virtual*/
size_t
DeepImageOutputTile::getNumSamplesAt (int x, int y) const
{
    const DeepLine* dl = getLine(y);
    if (!dl)
        return 0;
    x -= m_data_window.min.x;
    return (x < 0 || x >= (int)dl->samples_per_pixel.size())?0:dl->samples_per_pixel[x];
}


//----------------------------------------------------------


DeepImageOutputTile::DeepLine::DeepLine (uint32_t width,
                                         const ChannelSet& _channels) :
    channels(_channels)
{
    channel_arrays.resize(channels.size());
    samples_per_pixel.resize(width, 0);
}


void
DeepImageOutputTile::DeepLine::get (int xoffset,
                                    Dcx::DeepPixel& deep_pixel) const
{
#ifdef DCX_DEBUG_DEEPTILE
    assert(xoffset >= 0 && xoffset < samples_per_pixel.size());
#endif

    const uint32_t nSegments = samples_per_pixel[xoffset];

    deep_pixel.clear();
    if (nSegments == 0)
        return;
    deep_pixel.reserve(nSegments);

    const uint32_t foffset = floatOffset(xoffset);

    Dcx::DeepSegment ds;
    Dcx::Pixelf dp(deep_pixel.channels());
    float sp1, sp2;

    ChannelSet copy_channels(deep_pixel.channels());
    copy_channels &= channels;

    for (uint32_t i=0; i < nSegments; ++i)
    {
        ds.Zf = ds.Zb = 0.0f;
        sp1 = sp2 = 0.0f;
        ds.metadata.flags  = 0x0;

        int chan_index = 0;
        foreach_channel(z, copy_channels)
        {
            const float v = channel_arrays[chan_index][foffset + i];
            if (*z == Dcx::Chan_ZFront)
                ds.Zf = v;
            else if (*z == Dcx::Chan_ZBack)
                ds.Zb = v;
            else if (*z == Dcx::Chan_SpBits1)
                sp1 = v;
            else if (*z == Dcx::Chan_SpBits2)
                sp2 = v;
            else if (*z == Dcx::Chan_DeepFlags)
                ds.metadata.flags = (int)floorf(v);
            else
                dp[*z] = v;
            ++chan_index;
        }

        ds.metadata.spmask.fromFloat(sp1, sp2);
        if (ds.Zb < ds.Zf)
            ds.Zb = ds.Zf;
        ds.index = -1; // gets updated when added to deep pixel

        deep_pixel.append(ds, dp);
    }
}


void
DeepImageOutputTile::DeepLine::getMetadata(int xoffset,
                                           int sample,
                                           Dcx::DeepMetadata& metadata) const
{
    const uint32_t nSegments = samples_per_pixel[xoffset];
#ifdef DCX_DEBUG_DEEPTILE
    assert(xoffset >= 0 && xoffset < samples_per_pixel.size());
    assert(sample < nSegments);
#endif
    if (nSegments == 0)
        return;

    const uint32_t foffset = floatOffset(xoffset) + sample;

    float sp1=0.0f, sp2=0.0f;
    metadata.flags = 0x0;

    int chan_index = 0;
    foreach_channel(z, channels)
    {
        if (*z == Dcx::Chan_SpBits1)
            sp1 = channel_arrays[chan_index][foffset];
        else if (*z == Dcx::Chan_SpBits2)
            sp2 = channel_arrays[chan_index][foffset];
        else if (*z == Dcx::Chan_DeepFlags)
            metadata.flags = (int)floorf(channel_arrays[chan_index][foffset]);
        ++chan_index;
    }

    metadata.spmask.fromFloat(sp1, sp2);
}


void
DeepImageOutputTile::DeepLine::set (int xoffset,
                                    const Dcx::DeepPixel& deep_pixel)
{
#ifdef DCX_DEBUG_DEEPTILE
    assert(xoffset >= 0 && xoffset < samples_per_pixel.size());
#endif
    const size_t nWriteSegments = deep_pixel.size();
    if (nWriteSegments == 0)
    {
        clear(xoffset);
        return;
    }

    const uint32_t foffset = floatOffset(xoffset);
    const uint32_t nCurrSegments = samples_per_pixel[xoffset];
#ifdef DCX_DEBUG_DEEPTILE
    if (debug) std::cout << "  offset=" << offset << ", curr segments=" << nCurrSegments << std::endl;
#endif

    if (nCurrSegments < nWriteSegments)
    {
        // Add segments:
        const size_t nAdd = (nWriteSegments - nCurrSegments);
#ifdef DCX_DEBUG_DEEPTILE
        if (debug) std::cout << "  adding " << nAdd << " segments" << std::endl;
#endif
        int chan_index = 0;
        foreach_channel(z, channels)
        {
            FloatVec& values = channel_arrays[chan_index++];
#ifdef DCX_DEBUG_DEEPTILE
            assert(foffset <= values.size()); // shouldn't happen...
#endif
            // Round the memory reserve up in chunks to avoid constantly resizing/copying:
            if (values.capacity() < values.size()+nAdd)
                values.reserve(values.size() + (size_t)int(float(values.size())/1.5f));
            values.insert(values.begin() + foffset, nAdd, 0.0f);
        }
    }
    else if (nCurrSegments > nWriteSegments)
    {
        // Remove segments:
        const size_t nRemove = (nCurrSegments - nWriteSegments);
#ifdef DCX_DEBUG_DEEPTILE
        if (debug) std::cout << "  removing " << nRemove << " segments" << std::endl;
#endif
        int chan_index = 0;
        foreach_channel(z, channels)
        {
            FloatVec& values = channel_arrays[chan_index++];
            values.erase(values.begin() + foffset,
                         values.begin() + foffset + nRemove);
        }
    }

    samples_per_pixel[xoffset] = nWriteSegments;

    if (nWriteSegments > 0)
    {
        float sp1, sp2;
        // Copy segment channel data:
        for (uint32_t i=0; i < nWriteSegments; ++i)
        {
            const DeepSegment& segment = deep_pixel[i];
            const Pixelf& pixel = deep_pixel.getSegmentPixel(segment);
            segment.spMask().toFloat(sp1, sp2);

            float v;
            int chan_index = 0;
            foreach_channel(z, channels)
            {
                if (*z == Dcx::Chan_ZFront)
                    v = segment.Zf;
                else if (*z == Dcx::Chan_ZBack)
                    v = segment.Zb;
                else if (*z == Dcx::Chan_SpBits1)
                    v = sp1;
                else if (*z == Dcx::Chan_SpBits2)
                    v = sp2;
                else if (*z == Dcx::Chan_DeepFlags)
                    v = float(segment.flags());
                else if (deep_pixel.channels().contains(*z))
                    v = pixel[*z];
                else
                    v = 0.0f;
                channel_arrays[chan_index++][foffset + i] = v;
            }
        }
    }
}

void
DeepImageOutputTile::DeepLine::clear (int xoffset)
{
#ifdef DCX_DEBUG_DEEPTILE
    assert(xoffset >= 0 && xoffset < samples_per_pixel.size());
#endif
    const uint32_t nCurrSegments = samples_per_pixel[xoffset];
    if (nCurrSegments == 0)
        return;

    const uint32_t foffset = floatOffset(xoffset);

    int chan_index = 0;
    foreach_channel(z, channels)
    {
        FloatVec& values = channel_arrays[chan_index++];
        values.erase(values.begin() + foffset,
                     values.begin() + foffset + nCurrSegments);
    }
    samples_per_pixel[xoffset] = 0;
}


//----------------------------------------------------------


DeepImageOutputTile::DeepLine*
DeepImageOutputTile::createDeepLine (int y)
{
    if (y < m_data_window.min.y || y > m_data_window.max.y)
        return 0; // don't crash...  TODO: throw exception?
    y -= m_data_window.min.y;
    DeepLine* dl = m_deep_lines[y];
    if (!dl)
        m_deep_lines[y] = dl = new DeepLine(w(), channels());
    if (!dl)
        return 0; // don't crash... TODO: how to best handle memory alloc error...?
    return dl;
}


/*virtual*/
bool
DeepImageOutputTile::getDeepPixel (int x,
                                   int y,
                                   Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool /*debug*/
#endif
                                   ) const
{
    pixel.clear();
    if (!isActivePixel(x, y))
        return false;
    if (m_channels.empty())
        return true;

    DeepLine* dl = ((DeepImageOutputTile*)this)->createDeepLine(y);
    if (!dl)
        return false; // don't crash...

    dl->get(x - m_data_window.min.x, pixel);

    return true;
}

/*virtual*/
bool
DeepImageOutputTile::getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                        , bool /*debug*/
#endif
                                        ) const
{
    if (!isActivePixel(x, y))
        return false;
    if (m_channels.empty())
        return true;

    DeepLine* dl = ((DeepImageOutputTile*)this)->createDeepLine(y);
    if (!dl)
        return false; // don't crash...

    dl->getMetadata(x - m_data_window.min.x, sample, metadata);

    return true;
}



/*virtual*/
bool
DeepImageOutputTile::setDeepPixel (int x,
                                   int y,
                                   const Dcx::DeepPixel& deep_pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug
#endif
                                   )
{
    DeepLine* dl = createDeepLine(y);
    if (!dl || x < m_data_window.min.x || x > m_data_window.max.x)
        return false; // don't crash...

    const size_t nWriteSegments = deep_pixel.size();
#ifdef DCX_DEBUG_DEEPTILE
    if (debug)
    {
        std::cout << "setDeepPixel(" << x << ", " << y << ") segments=" << nWriteSegments;
        std::cout << ", numChannels=" << numChannels();
        m_channels.print(", tile channels=", std::cout, *m_channel_ctx);
        std::cout << std::endl;
    }
#endif

    // Copy DeepPixel data into packed DeepLine arrays (offseting x into array range):
    if (nWriteSegments == 0)
        dl->clear(x - m_data_window.min.x);
    else
        dl->set(x - m_data_window.min.x, deep_pixel);

    return true;
}


/*virtual*/
bool
DeepImageOutputTile::clearDeepPixel (int x,
                                     int y
#ifdef DCX_DEBUG_DEEPTILE
                                     , bool debug
#endif
                                     )
{
    DeepLine* dl = createDeepLine(y);
    if (!dl || x < m_data_window.min.x || x > m_data_window.max.x)
        return false; // don't crash...

#ifdef DCX_DEBUG_DEEPTILE
    if (debug) std::cout << "clearDeepPixel(" << x << ", " << y << ")" << std::endl;
#endif

    dl->clear(x - m_data_window.min.x); // Offset x into DeepLine array

    return true;
}


//
// Create an output deep file linked to this tile - destructive!
// Will allocate a new Imf::DeepScanLineOutputFile and assign its
// channels, destroying any current file.
//

/*virtual*/
void
DeepImageOutputTile::setOutputFile (const char* filename,
                                    Imf::LineOrder line_order)
{
    if (!filename || !filename[0] || m_filename == filename)
        return;
    m_filename = filename;

    // Flip line order to match Y-up mode:
    if (line_order == Imf::DECREASING_Y)
        line_order = (m_tile_yUp)?Imf::INCREASING_Y:Imf::DECREASING_Y;
    else if (line_order == Imf::INCREASING_Y)
        line_order = (m_tile_yUp)?Imf::DECREASING_Y:Imf::INCREASING_Y;
    else
        line_order = (m_tile_yUp)?Imf::DECREASING_Y:Imf::INCREASING_Y;

#ifdef DCX_DEBUG_DEEPTILE
std::cout << "DeepImageOutputTile::setOutputFileName('" << filename << "') line_order=" << line_order << std::endl;
#endif

    Imf::Header header(m_display_window,
                       m_data_window,
                       1.0,/*pixelAspectRatio*/
                       IMATH_NAMESPACE::V2f(0.0f, 0.0f), /*screenWindowCenter*/
                       1.0f, /*screenWindowWidth*/
                       line_order,
                       Imf::ZIPS_COMPRESSION); // Single-line zip for deep (TODO always...?)

    ChannelSet write_channels(m_channels);
    //write_channels -= Mask_SpMask8;
    //write_channels -= Mask_DeepFlags;

    foreach_channel(z, write_channels)
    {
        const ChannelAlias* c = getChannelAlias(*z);
        assert(c); // shouldn't happen...

        // Use the fileIOName for EXR output channel name:
        header.channels().insert(c->fileIOName(), Imf::Channel(c->fileIOPixelType()));

#ifdef DCX_DEBUG_DEEPTILE
std::cout << "  " << *z << ": ";
std::cout << " making channel '" << c->fileIOName() << "' of pixelType " << c->fileIOPixelType() << std::endl;
#endif
    }

#if 0
    if (m_channels.contains(Mask_SpMask8))
    {
        header.channels().insert("spmask.1", Imf::Channel(Imf::FLOAT));
        header.channels().insert("spmask.2", Imf::Channel(Imf::FLOAT));
#ifdef DCX_DEBUG_DEEPTILE
std::cout << "      making spmask channels" << std::endl;
#endif
    }
    if (m_channels.contains(Mask_DeepFlags))
    {
        header.channels().insert("spmask.flags", Imf::Channel(Imf::HALF));
#ifdef DCX_DEBUG_DEEPTILE
std::cout << "      making flags channels" << std::endl;
#endif
    }
#endif

    delete m_file;
    m_file = new Imf::DeepScanLineOutputFile(filename, header);
    deleteDeepLines();
}


//
// Write the DeepLine to the assigned output file.
// If flush_line is true the DeepLine memory is freed.
//

typedef std::vector<DeepImageOutputTile::HalfVec>  HalfSamples;
typedef std::vector<DeepImageOutputTile::FloatVec> FloatSamples;
typedef std::vector<DeepImageOutputTile::UintVec>  UintSamples;


/*virtual*/
void
DeepImageOutputTile::writeScanline (int y,
                                    bool flush_line
#ifdef DCX_DEBUG_DEEPTILE
                                    , bool debug
#endif
                                    )
{
#ifdef DCX_DEBUG_DEEPTILE
if (debug)
    std::cout << "DeepImageOutputTile::writeScanline(" << y << ")" << std::endl;
#endif

    if (!m_file || y < m_data_window.min.y || y > m_data_window.max.y)
        return; // don't crash...  TODO: throw exception?
    y -= m_data_window.min.y;
    const DeepLine* dl = m_deep_lines[y];
    if (!dl)
        return; // nothing to write

    // Unpack the floats to arrays of the appropriate pixel types and
    // assign the frambuffer slices to them:
    const size_t nChannels = m_channel_aliases.size();
#ifdef DCX_DEBUG_DEEPTILE
if (debug)
    std::cout << "  nChannels=" << nChannels;
#endif
    if (nChannels == 0)
        return;
    const size_t nPixels = dl->samples_per_pixel.size();
    assert(nPixels == this->w());

    // Unpacked sample data storage (only some of these actually get filled in):
    std::vector<HalfSamples>   half_samples(nChannels);
    std::vector<FloatSamples> float_samples(nChannels);
    std::vector<UintSamples>   uint_samples(nChannels);
    std::vector<PtrVec>           data_ptrs(nChannels);

    Imf::DeepFrameBuffer fb;
    fb.insertSampleCountSlice(Imf::Slice(Imf::UINT, 
                                         (char*)(dl->samples_per_pixel.data() - m_data_window.min.x),
                                         sizeof(uint32_t)/*xStride*/,
                                         0/*yStride*/));

    int chan_index = 0;
    size_t sample_stride = 0;
    foreach_channel(z, m_channels)
    {
        const ChannelAlias* c = getChannelAlias(*z);
        assert(c); // shouldn't happen...
#ifdef DCX_DEBUG_DEEPTILE
if (debug)
    std::cout << "  " << *z << ": '" << c->fileIOName() << "'";
#endif

        const float* IN = dl->channel_arrays[chan_index].data();

        PtrVec& ptrs = data_ptrs[chan_index];
        ptrs.resize(nPixels);

        switch (c->fileIOPixelType())
        {
            case Imf::HALF:
            {
                HalfSamples& value_lists = half_samples[chan_index];
                value_lists.resize(nPixels);
                for (size_t i=0; i < nPixels; ++i)
                {
                    const size_t nSamples = dl->samples_per_pixel[i];
                    HalfVec& samples = value_lists[i];
                    samples.reserve(nSamples);
                    for (size_t s=0; s < nSamples; ++s)
                        samples.push_back(half(*IN++));
                    ptrs[i] = samples.data(); // always point to valid data, even for 0 samples...
                }
                sample_stride = sizeof(half);
                break;
            }

            case Imf::FLOAT:
            {
                FloatSamples& value_lists = float_samples[chan_index];
                value_lists.resize(nPixels);
                for (size_t i=0; i < nPixels; ++i)
                {
                    const size_t nSamples = dl->samples_per_pixel[i];
                    FloatVec& samples = value_lists[i];
                    samples.reserve(nSamples);
                    for (size_t s=0; s < nSamples; ++s)
                        samples.push_back(*IN++);
                    ptrs[i] = samples.data(); // always point to valid data, even for 0 samples...
                }
                sample_stride = sizeof(float);
                break;
            }

            case Imf::UINT:
            {
                UintSamples& value_lists = uint_samples[chan_index];
                value_lists.resize(nPixels);
                for (size_t i=0; i < nPixels; ++i)
                {
                    const size_t nSamples = dl->samples_per_pixel[i];
                    UintVec& samples = value_lists[i];
                    samples.reserve(nSamples);
                    for (size_t s=0; s < nSamples; ++s, ++IN)
                        samples.push_back(int(floorf((*IN < 0.0f)?0.0f:*IN)));
                    ptrs[i] = samples.data();
                }
                sample_stride = sizeof(uint32_t);
                break;
            }

            default:
                assert(0); // TODO: throw exception instead?
                break;
        }

        fb.insert(c->fileIOName(), Imf::DeepSlice(c->fileIOPixelType(),
                                                  (char*)(ptrs.data() - m_data_window.min.x),
                                                  sizeof(void*)/*xStride*/,
                                                  0/*yStride*/,
                                                  sample_stride/*sampleStride*/));

        ++chan_index;
    }
#ifdef DCX_DEBUG_DEEPTILE
if (debug)
    std::cout << std::endl;
#endif

    // Write line to file:
    m_file->setFrameBuffer(fb);
    m_file->writePixels( 1/*nLines*/ );

    // Free the DeepLine:
    if (flush_line)
    {
        delete m_deep_lines[y];
        m_deep_lines[y] = 0;
    }
}

//
// Write entire tile to output file.
// If flush_tile is true all DeepLine memory is freed.
//

/*virtual*/
void
DeepImageOutputTile::writeTile (bool flush_tile)
{
    for (int y=m_data_window.min.y; y < m_data_window.max.y; ++y)
        writeScanline(y, flush_tile);
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
