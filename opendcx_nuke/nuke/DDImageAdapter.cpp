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
#include <DDImage/Thread.h>  // for Lock

#include <OpenDCX/DcxChannelContext.h>


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

//
// Shared channel context for all Nuke plugins.
//
// This is currently a global static because Nuke also has a global static list
// of DD::Image::Channels and Layers. If Nuke switches to using a context
// then this would change too.
//
// Be sure to lock before using methods that can create new channels
// like Dcx::ChannelContext::getChannel().
//

static Dcx::ChannelContext dcx_channel_context;


//
// Maps from DD::Image::Channel to Dcx::ChannelIdx
//

static std::map<DD::Image::Channel, Dcx::ChannelIdx>    dcx_channel_map;


//
// Make sure channel set conversions in engine() are thread safe by
// locking any writes to the static channel maps.
//

static DD::Image::Lock context_lock;



//
// Assign/get the DD::Image::Channels that are appropriate for OpenDCX.
//
// This calls DD::Image::getChannel() with 'sort=false' so that Nuke does
// not reorder the channels in the 'spmask' layer - we always want the order
// to be sp1, sp2, flags so that ChannelKnobs display correctly by default.
//

void
dcxGetSpmaskChannels (DD::Image::Channel& sp1,
                      DD::Image::Channel& sp2,
                      DD::Image::Channel& flags)
{
    // Call getChannel() with sort=false to avoid Nuke reordering these channels
    // in the layer:
    sp1   = DD::Image::getChannel(Dcx::spMask8Channel1Name, false/*sort*/);
    sp2   = DD::Image::getChannel(Dcx::spMask8Channel2Name, false/*sort*/);
    flags = DD::Image::getChannel(Dcx::flagsChannelName,    false/*sort*/);
}

// This should get called before any Op ctors so that the DCX channels are
// constructed properly even if getChannels() is called from an Op with
// sort=true:
struct GetDcxChannels {

    GetDcxChannels()
    {
        DD::Image::Channel sp1, sp2, flags;
        dcxGetSpmaskChannels(sp1, sp2, flags);
    }

};
static GetDcxChannels get_dcx_chans;


//
// Add new DD::Image::Channel by getting its full name and looking it
// up in the Dcx::ChannelContext.
// This may create a new Dcx::ChannelIdx.
//

Dcx::ChannelIdx
dcxAddChannel (DD::Image::Channel z)
{
    Dcx::ChannelIdx c = Dcx::Chan_Invalid;

    // Explicity test for some deep channels that won't auto-translate
    // to predefined Dcx::ChannelIdxs:
    if (z == DD::Image::Chan_DeepFront)
        c = Dcx::Chan_ZFront;
    else if (z == DD::Image::Chan_DeepBack)
        c = Dcx::Chan_ZBack;
    else
    {
        // Find the Dcx channel matching the full name of the DD::Image::Channel,
        // or create it:
        context_lock.lock();
        c = dcx_channel_context.getChannel(DD::Image::getName(z));
        dcx_channel_map[z] = c; // Remember new channel:
        context_lock.unlock();
    }
    assert(c != Dcx::Chan_Invalid); // shouldn't happen...

    //std::cout << "ddchan=" << (int)z << "'" << z << "' -> dcxchan=" << c << "'" << dcx_channel_context.getChannelFullName(c) << "'" << std::endl;
    return c;
}



//
// Convert a DD::Image::Channel to a Dcx::ChannelIdx.
// This may create a new Dcx::ChannelIdx.
//

Dcx::ChannelIdx
dcxChannel (DD::Image::Channel z)
{
    std::map<DD::Image::Channel, Dcx::ChannelIdx>::const_iterator it = dcx_channel_map.find(z);
    if (it != dcx_channel_map.end())
        return it->second;
    return dcxAddChannel(z); // not found, add it now
}


//
// Convert a DD::Image::ChannelSet to a Dcx::ChannelSet.
//
// This makes the conversion more speedy by checking against previous
// Channel conversions stored in a map.
//

Dcx::ChannelSet
dcxChannelSet (const DD::Image::ChannelSet& in)
{
    Dcx::ChannelSet out;
    foreach(z, in)
        out += dcxChannel(z);
    return out;
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
                         const DD::Image::Channel spmask_channel0,
                         const DD::Image::Channel spmask_channel1,
                         const DD::Image::Channel flags_channel,
                         Dcx::DeepPixel& out)
{
    out.clear();
    const unsigned nSamples = in.getSampleCount();
    if (nSamples == 0 || !in.channels().contains(DD::Image::Chan_DeepFront))
        return;
    out.reserve(nSamples);

    DD::Image::ChannelSet copy_channels(in.channels());
    copy_channels -= DD::Image::Mask_Deep;
    copy_channels -= DD::Image::Mask_Z;
    copy_channels -= spmask_channel0;
    copy_channels -= spmask_channel1;
    copy_channels -= flags_channel;
    out.setChannels(Dcx::dcxChannelSet(copy_channels));

    const bool have_Zb  = (in.channels().contains(DD::Image::Chan_DeepBack));

    // Copy each samples from the source DD::Image::DeepPixel:
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
        dcxGetDeepSampleMetadata(in, sample, spmask_channel0, spmask_channel1, flags_channel, ds.metadata);

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
                          const DD::Image::Channel spmask_channel0,
                          const DD::Image::Channel spmask_channel1,
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
    dcxGetDeepSampleMetadata(in, sample,
                             spmask_channel0, spmask_channel1,
                             flags_channel,
                             segment_out.metadata);

    // Copy the DD::Image::DeepPixel channel data into the Pixel:
    if (pixel_out)
    {
        DD::Image::ChannelSet copy_channels(in.channels());
        copy_channels -= DD::Image::Mask_Deep;
        copy_channels -= DD::Image::Mask_Z;
        copy_channels -= spmask_channel0;
        copy_channels -= spmask_channel1;
        copy_channels -= flags_channel;
        pixel_out->channels = dcxChannelSet(copy_channels);
        foreach(z, copy_channels)
            (*pixel_out)[dcxChannel(z)] = in.getUnorderedSample(sample, z);
    }

    return true;
}


//
// Extract subpixel mask and flags metadata from a DD::Image::DeepPixel sample.
//

void
dcxGetDeepSampleMetadata (const DD::Image::DeepPixel& in,
                          unsigned sample,
                          const DD::Image::Channel spmask_channel0,
                          const DD::Image::Channel spmask_channel1,
                          const DD::Image::Channel flags_channel,
                          Dcx::DeepMetadata& metadata_out)
{
    if (spmask_channel0 != DD::Image::Chan_Black && in.channels().contains(spmask_channel0) &&
        spmask_channel1 != DD::Image::Chan_Black && in.channels().contains(spmask_channel1))
        metadata_out.spmask.fromFloat(in.getUnorderedSample(sample, spmask_channel0),
                                      in.getUnorderedSample(sample, spmask_channel1));
    else
        metadata_out.spmask = Dcx::SpMask8::zeroCoverage; // default to zero-coverage (a legacy deep sample)

    // Extract flags from flags channel (convert from floating-point integer value):
    if (flags_channel != DD::Image::Chan_Black && in.channels().contains(flags_channel))
        metadata_out.flags.fromFloat(in.getUnorderedSample(sample, flags_channel));
    else
        metadata_out.flags.clearAll();
}


//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------


DDImageDeepPlane::DDImageDeepPlane (const DD::Image::Box& format,
                                    const DD::Image::Box& bbox,
                                    const DD::Image::ChannelSet& channels,
                                    const DD::Image::Channel spmask_channel0,
                                    const DD::Image::Channel spmask_channel1,
                                    const DD::Image::Channel flags_channel,
                                    WriteAccessMode write_access_mode) :
    Dcx::DeepTile(Imath::Box2i(Imath::V2i(format.x(),   format.y()),
                               Imath::V2i(format.r()-1, format.t()-1))/*display_window*/,
                  Imath::Box2i(Imath::V2i(bbox.x(),   bbox.y()),
                               Imath::V2i(bbox.r()-1, bbox.t()-1))/*data_window*/,
                  true/*sourceWindowsYup*/,
                  dcxChannelSet(channels),
                  dcx_channel_context,
                  write_access_mode,
                  true/*tileYup*/),
    m_flags_channel(flags_channel)
{
    m_spmask_channel[0] = spmask_channel0;
    m_spmask_channel[1] = spmask_channel1;
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
                                              const DD::Image::Channel spmask_channel0,
                                              const DD::Image::Channel spmask_channel1,
                                              const DD::Image::Channel flags_channel) :
    DDImageDeepPlane(format,
                     deep_in_plane->box(),
                     deep_in_plane->channels(),
                     spmask_channel0, spmask_channel1,
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
                            m_spmask_channel[0], m_spmask_channel[1],
                            m_flags_channel,
                            pixel);
    pixel.setXY(x, y);
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
                             m_spmask_channel[0], m_spmask_channel[1],
                             m_flags_channel,
                             metadata);
    return true;
}


//----------------------------------------------------------------------------------


DDImageDeepOutputPlane::DDImageDeepOutputPlane (const DD::Image::Box& format,
                                                DD::Image::DeepOutputPlane* deep_out_plane,
                                                const DD::Image::Channel spmask_channel0,
                                                const DD::Image::Channel spmask_channel1,
                                                const DD::Image::Channel flags_channel) :
    DDImageDeepPlane(format,
                     deep_out_plane->box(),
                     deep_out_plane->channels(),
                     spmask_channel0, spmask_channel1,
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
// Writes a DeepPixel to a sequential pixel in the DeepOutputPlane.
// Always returns true.
//

void
DDImageDeepOutputPlane::setDeepPixelSequential (const Dcx::DeepPixel& pixel)
{
    const size_t nSegments = pixel.size();
    assert(nSegments < 10000); // just in case...
    if (nSegments == 0)
    {
        m_out_plane->addHole();
        return;
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
            else if (z == m_spmask_channel[0])
            {
                out_pixel.push_back(sp1);
                continue;
            }
            else if (z == m_spmask_channel[1])
            {
                out_pixel.push_back(sp2);
                continue;
            }
            else if (z == m_flags_channel)
            {
                out_pixel.push_back(segment.flags().toFloat());
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
}


//
// Writes an empty DeepPixel (0 samples) to a sequential pixel in the DeepOutputPlane.
//

void
DDImageDeepOutputPlane::clearDeepPixelSequential ()
{
    m_out_plane->addHole();
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT
