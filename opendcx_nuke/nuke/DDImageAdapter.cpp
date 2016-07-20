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
/// @file DDImageAdapter.cpp


#include "DDImageAdapter.h"

#include <OpenDCX/DcxChannelContext.h>


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

//
// Shared channel context for all Nuke plugins.
// Assumes access is single-threaded (usually from _validate() calls)
//
static Dcx::ChannelContext dcx_channel_context;


//
// Maps from DD::Image::Channel to Dcx::ChannelIdx and Dcx::ChannelAlias:
//
static std::map<DD::Image::Channel, Dcx::ChannelIdx> dcx_channel_map;
static std::map<DD::Image::Channel, Dcx::ChannelAlias*> dcx_channelalias_map;

//
// Speed up repeated ChannelSet conversions by using a hash map:
//
static std::vector<Dcx::ChannelSet*>    dcx_channelset_list;
static std::map<U64, Dcx::ChannelSet*>  dcx_channelset_map;



//
// Convert a DD::Image::ChannelSet to a Dcx::ChannelSet.
//
// This attempts to make the conversion more speedy by checking against previous
// ChannelSet conversions stored in a map, the idea being that unique ChannelSet
// combinations are relatively few.
//
// TODO: However, building the hash to test still requires the DD::Image::ChannelSet to
// step through its channels appending to the hash, so in the end it may not be any
// faster than just stepping through the channels in this method and using
// dcx_channel_map lookups instead to build up the output Dcx::ChannelSet...
// What's faster, a bunch of Hash append calls or a bunch of map lookups...?
//

Dcx::ChannelSet
dcxChannelSet (const DD::Image::ChannelSet& in)
{
#if 1
    // Simpler version that uses just the channel->channel map:
    Dcx::ChannelSet out;
    foreach(z, in)
    {
        std::map<DD::Image::Channel, Dcx::ChannelIdx>::const_iterator it = dcx_channel_map.find(z);
        if (it != dcx_channel_map.end())
        {
            out += it->second;
            continue;
        }
        // Add new DD::Image::Channel by getting its full name - this
        // may create a new Dcx::ChannelIdx.
        //
        // Explicity test for some deep channels that won't auto-translate
        // to predefined Dcx::ChannelIdxs:
        Dcx::ChannelIdx c = Dcx::Chan_Invalid;
        if (z == DD::Image::Chan_DeepFront)
            c = Dcx::Chan_ZFront;
        else if (z == DD::Image::Chan_DeepBack)
            c = Dcx::Chan_ZBack;
        else
            c = dcx_channel_context.getChannel(DD::Image::getName(z));
        if (c != Dcx::Chan_Invalid)
        {
            out += c;
            dcx_channel_map[z] = c;
            // Also add it to the aliases map:
            Dcx::ChannelAlias* alias = dcx_channel_context.findChannelAlias(c);
            assert(alias); // shouldn't happen...
            dcx_channelalias_map[z] = alias;
            //std::cout << "ddchan=" << (int)z << "'" << z << "' -> dcxchan=" << c << "'" << dcx_channel_context.getChannelFullName(c) << "'" << std::endl;
        }
    }
    return out;
#else
    // First check if set has already been converted:
    DD::Image::Hash hash;
    in.append(hash);
    std::map<U64, Dcx::ChannelSet*>::const_iterator it = dcx_channelset_map.find(hash.value());
    if (it != dcx_channelset_map.end())
        return *it->second;

    // Not found, create a new set:
    Dcx::ChannelSet* new_set = new Dcx::ChannelSet();
    foreach(z, in)
    {
        const Dcx::ChannelIdx c = dcx_channel_context.getChannel(DD::Image::getName(z));
        if (c != Dcx::Chan_Invalid)
        {
            *new_set += c;
            dcx_channel_map[z] = c;
            //std::cout << "ddchan=" << (int)z << "'" << z << "' -> dcxchan=" << c << "'" << dcx_channel_context.getChannelFullName(c) << "'" << std::endl;
        }
    }
    dcx_channelset_list.push_back(new_set);
    dcx_channelset_map[hash.value()] = new_set;

    return *new_set;
#endif
}


//
// 
//

Dcx::ChannelAliasPtrSet
dcxChannelAliasPtrSet (const DD::Image::ChannelSet& in)
{
    Dcx::ChannelAliasPtrSet out;
    foreach(z, in)
    {
        std::map<DD::Image::Channel, Dcx::ChannelAlias*>::const_iterator it = dcx_channelalias_map.find(z);
        if (it != dcx_channelalias_map.end())
        {
            assert(it->second); // shouldn't happen...
            out.insert(it->second);
            continue;
        }
        // Add new DD::Image::Channel by getting its full name - this
        // may create a new Dcx::ChannelIdx.
        //
        // Explicity test for some deep channels that won't auto-translate
        // to predefined Dcx::ChannelIdxs:
        Dcx::ChannelIdx c = Dcx::Chan_Invalid;
        if (z == DD::Image::Chan_DeepFront)
            c = Dcx::Chan_ZFront;
        else if (z == DD::Image::Chan_DeepBack)
            c = Dcx::Chan_ZBack;
        else
            c = dcx_channel_context.getChannel(DD::Image::getName(z));
        if (c != Dcx::Chan_Invalid)
        {
            dcx_channel_map[z] = c;
            // Also add it to the aliases map:
            Dcx::ChannelAlias* alias = dcx_channel_context.findChannelAlias(c);
            assert(alias); // shouldn't happen...
            dcx_channelalias_map[z] = alias;
            out.insert(alias);
            //std::cout << "ddchan=" << (int)z << "'" << z << "' -> dcxchan=" << c << "'" << dcx_channel_context.getChannelFullName(c) << "'" << std::endl;
        }
    }
    return out;
}


//
// 
//

Dcx::ChannelIdx
dcxChannel (DD::Image::Channel z)
{
    std::map<DD::Image::Channel, Dcx::ChannelIdx>::const_iterator it = dcx_channel_map.find(z);
    if (it == dcx_channel_map.end())
        return Dcx::Chan_Invalid;
    return it->second;
}



Imath::M44f
dcxMatrix4ToM44f(const DD::Image::Matrix4& m)
{
    float m44[4][4];
#if 1
    memcpy(m44[0], m.array(), 16*sizeof(float));
#else
    for (unsigned j=0; j < 4; ++j)
        for (unsigned i=0; i < 4; ++i)
            m44[j][i] = m[j][i];
#endif
    return Imath::M44f(m44);
}


//
// Copy the contents of a DD::Image::DeepPixel to a Dcx::DeepPixel.
//

void
dcxCopyDDImageDeepPixel (const DD::Image::DeepPixel& in,
                         Dcx::SpMaskMode spmask_mode,
                         const std::vector<DD::Image::Channel>& spmask_chan_list,
                         const DD::Image::Channel flags_channel,
                         Dcx::DeepPixel& out)
{
    out.clear();
    const unsigned nSamples = in.getSampleCount();
    if (nSamples == 0 || !in.channels().contains(DD::Image::Chan_DeepFront))
        return;
    // Copy the samples from the source DD::Image::DeepPixel:
    out.reserve(nSamples);

    DD::Image::ChannelSet copy_channels(in.channels());
    copy_channels -= DD::Image::Mask_Deep;
    copy_channels -= DD::Image::Mask_Z;
    for (unsigned i=0; i < spmask_chan_list.size(); ++i)
        copy_channels -= spmask_chan_list[i];
    copy_channels -= flags_channel;
    out.setChannels(Dcx::dcxChannelSet(copy_channels));

    const bool have_Zb  = (in.channels().contains(DD::Image::Chan_DeepBack));

    float Zf, Zb;
    Dcx::DeepSegment ds;
    for (unsigned sample=0; sample < nSamples; ++sample)
    {
        Zf = in.getUnorderedSample(sample, DD::Image::Chan_DeepFront);
        // Skip samples with negative, infinite or nan Zfront:
        if (Zf < 0.0f || isinf(Zf) || isnan(Zf))
            continue;

        if (have_Zb)
        {
            Zb = in.getUnorderedSample(sample, DD::Image::Chan_DeepBack);
            // Clamp Zback to reasonable values - allow infinity:
            if (isnan(Zb) || Zb < Zf)
                Zb = Zf;
            ds.setDepths(Zf, Zb);
        }
        else
            ds.setDepths(Zf, Zf);

        ds.index = sample;

        // Extract metadata from input channels:
        dcxGetDeepSampleMetadata(in, sample, spmask_mode, spmask_chan_list, flags_channel, ds.metadata);

        // Add segment and copy pixel data:
        const unsigned dsindex = out.append(ds);
        // Copy the DD::Image::DeepPixel channel data into the Dcx::Pixel:
        Dcx::Pixelf& p = out.getSegmentPixel(dsindex);
        foreach(z, copy_channels)
            p[dcxChannel(z)] = in.getUnorderedSample(sample, z);
    }
}


//
// Copy one DD::Image::DeepPixel sample.  Returns true on success.
// This method probes the spmask channels for valid metadata info and sets
// the DeepMetadata for the segment.
//
// The segment's channel data index is left unassigned!
//

bool
dcxCopyDDImageDeepSample (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          Dcx::SpMaskMode spmask_mode,
                          const std::vector<DD::Image::Channel>& spmask_chan_list,
                          const DD::Image::Channel flags_channel,
                          Dcx::DeepSegment& segment_out,
                          Dcx::Pixelf* pixel_out)
{
    if (in.getSampleCount() == 0 || sample >= in.getSampleCount() ||
         !in.channels().contains(DD::Image::Chan_DeepFront))
        return false;

    // Skip samples with negative, inf or nan Zfront:
    const float Zf = in.getUnorderedSample(sample, DD::Image::Chan_DeepFront);
    if (Zf < 0.0f || isinf(Zf) || isnan(Zf))
        return false;

    segment_out.index = -1; // default to unassigned!
    // Possibly update ZBack:
    float Zb = Zf;
    if (in.channels().contains(DD::Image::Chan_DeepBack))
    {
        Zb = in.getUnorderedSample(sample, DD::Image::Chan_DeepBack);
        if (Zb < Zf || isnan(Zb))
            Zb = Zf; // Clamp Zback to reasonable values, allow infinity
    }
    segment_out.setDepths(Zf, Zb);

    // Extract metadata from input channels:
    dcxGetDeepSampleMetadata(in, sample, spmask_mode, spmask_chan_list, flags_channel, segment_out.metadata);

    // Copy the DD::Image::DeepPixel channel data into the Pixel:
    if (pixel_out)
    {
        DD::Image::ChannelSet copy_channels(in.channels());
        copy_channels -= DD::Image::Mask_Deep;
        copy_channels -= DD::Image::Mask_Z;
        for (unsigned i=0; i < spmask_chan_list.size(); ++i)
            copy_channels -= spmask_chan_list[i];
        copy_channels -= flags_channel;
        pixel_out->channels = dcxChannelSet(copy_channels);
        foreach(z, copy_channels)
            (*pixel_out)[dcxChannel(z)] = in.getUnorderedSample(sample, z);
    }

    return true;
}


//
// Extract subpixel mask and flags metadata from input 'spmask' layer.
//

void
dcxGetDeepSampleMetadata (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          Dcx::SpMaskMode spmask_mode,
                          const std::vector<DD::Image::Channel>& spmask_chan_list,
                          const DD::Image::Channel flags_channel,
                          Dcx::DeepMetadata& metadata_out)
{
    metadata_out.spmask = Dcx::SpMask8::zeroCoverage; // default to zero-coverage (legacy deep)
    metadata_out.flags  = 0x0;

    if (spmask_mode == Dcx::SPMASK_AUTO)
    {
        if (spmask_chan_list.size() == 2 &&
            in.channels().contains(spmask_chan_list[0]) && in.channels().contains(spmask_chan_list[1]))
        {
           metadata_out.spmask.fromFloat(in.getUnorderedSample(sample, spmask_chan_list[0]),
                                         in.getUnorderedSample(sample, spmask_chan_list[1]));

        }
        else if (spmask_chan_list.size() == 1 && in.channels().contains(spmask_chan_list[0]))
        {
            //metadata_out.spmask = Dcx::spMask4From1Float(in.getUnorderedSample(sample, spmask_chan_list[0]));

        }
        else
        {
            metadata_out.spmask = Dcx::SpMask8::zeroCoverage; // default to zero coverage (legacy data)
        }

    }
    else if (spmask_mode == Dcx::SPMASK_8x8 && spmask_chan_list.size() == 2 &&
             in.channels().contains(spmask_chan_list[0]) && in.channels().contains(spmask_chan_list[1]))
    {
        metadata_out.spmask.fromFloat(in.getUnorderedSample(sample, spmask_chan_list[0]),
                                      in.getUnorderedSample(sample, spmask_chan_list[1]));

    }
    else if (spmask_mode == Dcx::SPMASK_4x4 && spmask_chan_list.size() == 1 &&
             in.channels().contains(spmask_chan_list[0]))
    {
        //metadata_out.spmask = Dcx::spMask4From1Float(in.getUnorderedSample(sample, spmask_chan_list[0]));

    }

    // Extract flags from flags channel (convert from floating-point integer value):
    if (in.channels().contains(flags_channel))
        metadata_out.flags = (Dcx::DeepFlag)floorf(in.getUnorderedSample(sample, flags_channel));
}


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

DDImageDeepPlane::DDImageDeepPlane (const DD::Image::Box& format,
                                    const DD::Image::Box& bbox,
                                    const DD::Image::ChannelSet& channels,
                                    Dcx::SpMaskMode spmask_mode,
                                    const std::vector<DD::Image::Channel>& spmask_chan_list,
                                    const DD::Image::Channel flags_channel,
                                    WriteAccessMode write_access_mode) :
    Dcx::DeepTile(Imath::Box2i(Imath::V2i(format.x(),   format.y()),
                               Imath::V2i(format.r()-1, format.t()-1))/*display_window*/,
                  Imath::Box2i(Imath::V2i(bbox.x(),   bbox.y()),
                               Imath::V2i(bbox.r()-1, bbox.t()-1))/*data_window*/,
                  true/*sourceWindowsYup*/,
                  dcxChannelAliasPtrSet(channels),
                  dcx_channel_context,
                  write_access_mode,
                  true/*tileYup*/),
    m_spmask_mode(spmask_mode),
    m_spmask8_chan_list(spmask_chan_list),
    m_flags_channel(flags_channel)
{
    //
}

/*virtual*/
size_t
DDImageDeepPlane::getNumSamplesAt (int x, int y) const
{
    return 0;
}

/*virtual*/
bool
DDImageDeepPlane::getDeepPixel (int x,
                                int y,
                                Dcx::DeepPixel& pixel) const
{
    return false;
}

/*virtual*/
bool
DDImageDeepPlane::getSampleMetadata (int x,
                                     int y,
                                     size_t sample,
                                     Dcx::DeepMetadata& metadata) const
{
    return false;
}


//----------------------------------------------------------------------------------

DDImageDeepInputPlane::DDImageDeepInputPlane (const DD::Image::Box& format,
                                              const DD::Image::DeepPlane* deep_in_plane,
                                              Dcx::SpMaskMode spmask_mode,
                                              const std::vector<DD::Image::Channel>& spmask_chan_list,
                                              const DD::Image::Channel flags_channel) :
    DDImageDeepPlane(format,
                     deep_in_plane->box(),
                     deep_in_plane->channels(),
                     spmask_mode,
                     spmask_chan_list,
                     flags_channel,
                     Dcx::DeepTile::WRITE_DISABLED),
    m_in_plane(deep_in_plane)
{
    //
}

//
// Returns the number of deep samples at pixel x,y.
//

/*virtual*/
size_t
DDImageDeepInputPlane::getNumSamplesAt (int x, int y) const
{
    return 0;
}


//
// Reads deep samples from a pixel-space location (x, y) into a deep pixel.
// If xy is out of bounds the deep pixel is left empty and false is returned.
//

/*virtual*/
bool
DDImageDeepInputPlane::getDeepPixel (int x,
                                     int y,
                                     Dcx::DeepPixel& pixel) const
{
    dcxCopyDDImageDeepPixel(m_in_plane->getPixel(y, x),
                            m_spmask_mode,
                            m_spmask8_chan_list,
                            m_flags_channel,
                            pixel);
    return true;
}


/*virtual*/
bool
DDImageDeepInputPlane::getSampleMetadata (int x,
                                          int y,
                                          size_t sample,
                                          Dcx::DeepMetadata& metadata) const
{
    dcxGetDeepSampleMetadata(m_in_plane->getPixel(y, x),
                             sample,
                             m_spmask_mode,
                             m_spmask8_chan_list,
                             m_flags_channel,
                             metadata);
    return true;
}


//----------------------------------------------------------------------------------


DDImageDeepOutputPlane::DDImageDeepOutputPlane (const DD::Image::Box& format,
                                                DD::Image::DeepOutputPlane* deep_out_plane,
                                                Dcx::SpMaskMode spmask_mode,
                                                const std::vector<DD::Image::Channel>& spmask_chan_list,
                                                const DD::Image::Channel flags_channel) :
    DDImageDeepPlane(format,
                     deep_out_plane->box(),
                     deep_out_plane->channels(),
                     spmask_mode,
                     spmask_chan_list,
                     flags_channel,
                     Dcx::DeepTile::WRITE_SEQUENTIAL),
    m_out_plane(deep_out_plane)
{
    //
}

//
// Returns the number of deep samples at pixel x,y.
//

/*virtual*/
size_t
DDImageDeepOutputPlane::getNumSamplesAt (int x, int y) const
{
    return 0;
}


//
// Reads deep samples from a pixel-space location (x, y) into a deep pixel.
// If xy is out of bounds the deep pixel is left empty and false is returned.
//

/*virtual*/
bool
DDImageDeepOutputPlane::getDeepPixel (int x,
                                      int y,
                                      Dcx::DeepPixel& pixel) const
{
    return false;
}


/*virtual*/
bool
DDImageDeepOutputPlane::getSampleMetadata (int x,
                                           int y,
                                           size_t sample,
                                           Dcx::DeepMetadata& metadata) const
{
    return false;
}


//
// Writes a DeepPixel to a pixel-space location (x, y) in the DeepOutputPlane.
// If xy is out of bounds or the image can't be written to, the deep pixel
// is left empty and false is returned.
//

/*virtual*/
bool
DDImageDeepOutputPlane::setDeepPixel (int,/*x ignored*/
                                      int,/*y ignored*/
                                      const Dcx::DeepPixel& pixel)
{
    const size_t nSegments = pixel.size();
    assert(nSegments < 10000); // just in case...
    if (nSegments == 0)
    {
        m_out_plane->addHole();
        return true;
    }

    // Copy Dcx::DeepSegments:
    DD::Image::ChannelSet copy_channels(m_out_plane->channels());
    DD::Image::DeepOutPixel out_pixel;
    out_pixel.reserve(nSegments*m_out_plane->channels().size());
    float sp1, sp2;

    for (size_t i=0; i < nSegments; ++i)
    {
        const DeepSegment& segment = pixel[i];
        const Pixelf& sgpixel = pixel.getSegmentPixel(segment);
        segment.spMask().toFloat(sp1, sp2);

        // Must output floats in DD::Image::ChannelSet order:
        foreach(z, copy_channels)
        {
            // Handle Zs & segment metadata explicitly:
            if (z == DD::Image::Chan_DeepFront)
            {
                out_pixel.push_back(segment.Zf);
                continue;
            }
            else if (z == DD::Image::Chan_DeepBack)
            {
                out_pixel.push_back(segment.Zb);
                continue;
            }
            else if (z == m_spmask8_chan_list[0])
            {
                out_pixel.push_back(sp1);
                continue;
            }
            else if (z == m_spmask8_chan_list[1])
            {
                out_pixel.push_back(sp2);
                continue;
            }
            else if (z == m_flags_channel)
            {
                out_pixel.push_back(float(segment.flags()));
                continue;
            }

            // Handle color channels / AOVs:
            Dcx::ChannelIdx dcx_chan = dcxChannel(z);
            if (pixel.channels().contains(dcx_chan))
                out_pixel.push_back(sgpixel[dcx_chan]);
            else
                out_pixel.push_back(0.0f);
        }

    }

    m_out_plane->addPixel(out_pixel);

    return true;
}


//
// Writes an empty DeepPixel (0 samples) to the DeepOutputPlane.
// Write access is sequential so both x/y args are ignored.
//

/*virtual*/
bool
DDImageDeepOutputPlane::clearDeepPixel (int,/*x ignored*/
                                        int /*y ignored*/)
{
    m_out_plane->addHole();
    return true;
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
