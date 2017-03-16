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
/// @file DeepSubpixelMask.cpp

#include <DDImage/DeepFilterOp.h>
#include <DDImage/Knobs.h>
#include <DDImage/Convolve.h>

#include <OpenDCX/DcxDeepPixel.h>

#include "DDImageAdapter.h" // for spmask nuke channel assignments


using namespace DD::Image;

//
//  DeepSubpixelMask
//
//      An example Nuke plugin which displays a deep sample's subpixel mask pattern
//      and allows it to be set.
//      This plugin is intended primarily for debugging purposes.
//

//----------------------------------------------------------------------------------------

enum { PATTERN_GET, PATTERN_SET };
const char* const pattern_modes[] = { "get", "set", 0 };

//----------------------------------------------------------------------------------------

/*!
*/
class DeepSubpixelMask : public DeepFilterOp {
    float                   k_pos[2];               //!< Pixel to sample
    int                     k_sample;               //!< Which sample to sample
    DD::Image::Channel      k_color_channel[4];     //!< Color layer to sample
    DD::Image::Channel      k_spmask_channel[2];    //!< Per-sample subpixel mask channels
    DD::Image::Channel      k_flags_channel;        //!< Per-sample flags channel
    int                     k_mode;
    bool                    k_set_all_samples;      //!< In set mode apply pattern to all samples
    //
    int                     k_num_samples;          //!< Num samples for selected pixel
    ConvolveArray           k_spmask_pattern;       //!< For pattern knob
    float                   k_spmask_array[Dcx::SpMask8::numBits]; //!< Pattern data
    double                  k_Zf_sampled;           //!< Sampled Zf
    double                  k_Zb_sampled;           //!< Sampled Zb
    const char*             k_flags_sampled;        //!< Sampled flags string
    double                  k_color_sampled[4];     //!< Sampled color values
    //
    Dcx::DeepMetadata       m_dpmeta;               //!< Derived deep metadata (subpixel mask, flags)

public:
    static const Description description;
    /*virtual*/ const char* Class() const { return description.name; }
    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "Get or set the subpixel mask at a pixel location and sample.\n"
        "This is primarily for debugging purposes.";
    }

    DeepSubpixelMask(Node* node) : DeepFilterOp(node), k_spmask_pattern() {
        // Get OpenDCX standard channels assigned in the correct order:
        Dcx::dcxGetSpmaskChannels(k_spmask_channel[0], k_spmask_channel[1], k_flags_channel);
        //
        k_pos[0]            = k_pos[1] = 0.0f;
        k_sample            = 0;
        k_color_channel[0]  = Chan_Red;
        k_color_channel[1]  = Chan_Green;
        k_color_channel[2]  = Chan_Blue;
        k_color_channel[3]  = Chan_Alpha;
        k_mode              = PATTERN_GET;
        k_set_all_samples   = false;
        k_num_samples       = 0;
        k_spmask_pattern.width  = Dcx::SpMask8::width;
        k_spmask_pattern.height = Dcx::SpMask8::height;
        memset(k_spmask_array, 0, Dcx::SpMask8::numBits*sizeof(float));
        k_spmask_pattern.array  = k_spmask_array;
        k_Zf_sampled = k_Zb_sampled = 0.0;
        k_flags_sampled = "";
        k_color_sampled[0] = k_color_sampled[1] = k_color_sampled[2] = k_color_sampled[3] = 0.0;
    }

    /*virtual*/ Op* op() { return this; }

    /*! Force knobs to update if the input changes in any way.
    */
    /*virtual*/
    void append(Hash& hash) {
        if (k_mode == PATTERN_GET && input0())
            hash = input0()->op()->hash();
    }

    /*virtual*/
    void knobs(Knob_Callback f) {
        XY_knob(f, k_pos, "pos");
        Input_Channel_knob(f, k_color_channel, 4/*nChans*/, 0/*input*/, "channels", "color channels");
            SetFlags(f, Knob::EARLY_STORE);
            Tooltip(f, "The color channels to display");
        Input_Channel_knob(f, k_spmask_channel, 2/*nChans*/, 0/*input*/, "spmask_channels", "spmask channels");
            SetFlags(f, Knob::EARLY_STORE);
            Tooltip(f, "Channels which contain the per-sample spmask data. Two channels are required for an 8x8 mask.");
        Input_Channel_knob(f, &k_flags_channel, 1/*nChans*/, 0/*input*/, "flags_channel", "flags");
            SetFlags(f, Knob::EARLY_STORE);
            ClearFlags(f, Knob::STARTLINE);
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channel which contains the per-sample flag data.");
        Enumeration_knob(f, &k_mode, pattern_modes, "mode", "mode");
            SetFlags(f, Knob::EARLY_STORE);
        Bool_knob(f, &k_set_all_samples, "set_all_samples", "set all samples");
            ClearFlags(f, Knob::STARTLINE);
        Divider(f);
        Int_knob(f, &k_sample, "sample", "do sample");
            Tooltip(f, "Sample to get or set the subpixel mask on.");
        Int_knob(f, &k_num_samples, "num_samples", " out of ");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::STARTLINE);
        Text_knob(f, " samples");
            ClearFlags(f, Knob::STARTLINE);
        Newline(f);
        //
        Double_knob(f, &k_Zf_sampled, "Zf", "Zf");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::SLIDER);
        Double_knob(f, &k_Zb_sampled, "Zb", "Zb");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
        String_knob(f, &k_flags_sampled, "flags", "flags");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_MULTIVIEW);
            ClearFlags(f, Knob::STARTLINE);
        Newline(f);
        //
#if 0
        AColor_knob(f, k_color_sampled, "colors", "color");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::SLIDER);
#else
        Double_knob(f, &k_color_sampled[0], "color0", "color");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::SLIDER);
        Double_knob(f, &k_color_sampled[1], "color1", "");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
        Double_knob(f, &k_color_sampled[2], "color2", "");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
        Double_knob(f, &k_color_sampled[3], "color3", "");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
#endif
        //
        Array_knob(f, &k_spmask_pattern, Dcx::SpMask8::width, Dcx::SpMask8::height, "spmask", "pattern");
            SetFlags(f, Knob::EARLY_STORE | Knob::NO_ANIMATION);
            Tooltip(f, "In set mode any non-zero value sets the bit.\n"
                        "Note: if all bits are set to zero (0) the spmask is disabled and it is "
                        "interpreted as a legacy deep sample with full pixel coverage (i.e. all "
                        "bits on)");
    }

    /*virtual*/
    int knob_changed(Knob* k) {
        DeepFilterOp::knob_changed(k);

        // Any change should force an update:
        bool sampled_pixel=false, sampled_spmask=false, sampled_flags=false;
        updateSampleKnobs(sampled_pixel, sampled_spmask, sampled_flags);

        knob("num_samples")->enable(sampled_pixel);
        knob("Zf")->enable(sampled_pixel);
        knob("Zb")->enable(sampled_pixel);
        knob("flags")->enable(sampled_flags);
#if 0
        knob("colors")->enable(sampled_pixel);
#else
        knob("color0")->enable(sampled_pixel);
        knob("color1")->enable(sampled_pixel);
        knob("color2")->enable(sampled_pixel);
        knob("color3")->enable(sampled_pixel);
#endif
        knob("spmask")->enable(sampled_spmask || k_mode == PATTERN_SET);

        return 1; // make sure we get called again
    }

    /*virtual*/
    bool updateUI(const OutputContext& context) {
        if (k_mode == PATTERN_GET) {
            bool sampled_pixel=false, sampled_spmask=false, sampled_flags=false;
            updateSampleKnobs(sampled_pixel, sampled_spmask, sampled_flags);
        }
        return true;
    }

    /*virtual*/
    void _validate(bool for_real) {
        DeepFilterOp::_validate(for_real);

        if (k_mode == PATTERN_SET) {
            // Flip the float array vertically when filling in the spmask:
            m_dpmeta.spmask = Dcx::SpMask8(0x0);
            for (int sp_y=0; sp_y < Dcx::SpMask8::height; ++sp_y) {
                Dcx::SpMask8 sp = (Dcx::SpMask8(0x01) << (sp_y*Dcx::SpMask8::width));
                for (int sp_x=0; sp_x < Dcx::SpMask8::width; ++sp_x, sp <<= 1) {
                    if (k_spmask_array[sp_x + (Dcx::SpMask8::height-sp_y-1)*Dcx::SpMask8::width] >= 0.5f)
                        m_dpmeta.spmask |= sp;
                }
            }
        }

        // Output subpixel mask channels:
        DD::Image::ChannelSet out_channels(_deepInfo.channels());
        if (k_mode == PATTERN_SET) {
            out_channels += k_spmask_channel[0];
            out_channels += k_spmask_channel[1];
        }

        _deepInfo = DeepInfo(_deepInfo.formats(), _deepInfo.box(), out_channels);
    }

    /*! Expand the deep request to include the sample location and spmask/flag channels.
    */
    /*virtual*/
    void getDeepRequests(Box bbox, const DD::Image::ChannelSet& channels, int count, std::vector<RequestData>& requests) {
        bbox.merge(int(k_pos[0]), int(k_pos[1]));
        DD::Image::ChannelSet input_channels(channels);
        input_channels += Mask_Deep; // just in case...
        input_channels += k_spmask_channel[0];
        input_channels += k_spmask_channel[1];
        input_channels += k_flags_channel;
        input_channels += k_color_channel[0];
        input_channels += k_color_channel[1];
        input_channels += k_color_channel[2];
        input_channels += k_color_channel[3];
        DeepFilterOp::getDeepRequests(bbox, input_channels, count, requests);
    }

    /*! Get the input deep samples for the selected pixel and copy or set
        the data depending on the mode.
    */
    void updateSampleKnobs(bool& sampled_pixel, bool& sampled_spmask, bool& sampled_flags) {
        sampled_pixel = sampled_spmask = sampled_flags = false;
        if (!input0())
           return; // need input!

        const int sampleX = int(k_pos[0]);
        const int sampleY = int(k_pos[1]);

        input0()->validate(true);
        DeepInfo deepInfo = input0()->deepInfo();
        Box box(sampleX, sampleY, sampleX+1, sampleY+1);

        DD::Image::ChannelSet get_channels(Mask_None);
        if (deepInfo.channels().contains(Mask_Deep)          ) get_channels += Mask_Deep;
        if (deepInfo.channels().contains(k_spmask_channel[0])) get_channels += k_spmask_channel[0];
        if (deepInfo.channels().contains(k_spmask_channel[1])) get_channels += k_spmask_channel[1];
        if (deepInfo.channels().contains(k_flags_channel    )) get_channels += k_flags_channel;
        if (deepInfo.channels().contains(k_color_channel[0] )) get_channels += k_color_channel[0];
        if (deepInfo.channels().contains(k_color_channel[1] )) get_channels += k_color_channel[1];
        if (deepInfo.channels().contains(k_color_channel[2] )) get_channels += k_color_channel[2];
        if (deepInfo.channels().contains(k_color_channel[3] )) get_channels += k_color_channel[3];

        input0()->deepRequest(box, get_channels);

        k_num_samples = 0;
        k_Zf_sampled = k_Zb_sampled = 0.0;
        k_flags_sampled = "";
        k_color_sampled[0] = k_color_sampled[1] = k_color_sampled[2] = k_color_sampled[3] = 0.0;
        std::string flags_str;
        memset(k_spmask_array, 0, Dcx::SpMask8::numBits*sizeof(float));

        if (!get_channels.empty() &&
            sampleX >= deepInfo.x() && sampleX < deepInfo.r() &&
            sampleY >= deepInfo.y() && sampleY < deepInfo.t()) {

            DD::Image::DeepPlane in_plane;
            if (input0()->deepEngine(sampleY, sampleX, sampleX+1, get_channels, in_plane)) {

                DD::Image::DeepPixel in_pixel = in_plane.getPixel(sampleY, sampleX);
                k_num_samples = in_pixel.getSampleCount();
                if (k_sample >= 0 && k_sample < k_num_samples) {
                    const unsigned get_sample = k_num_samples - k_sample - 1;
                    k_Zf_sampled = in_pixel.getOrderedSample(get_sample, Chan_DeepFront);
                    k_Zb_sampled = in_pixel.getOrderedSample(get_sample, Chan_DeepBack);
                    k_color_sampled[0] = in_pixel.getOrderedSample(get_sample, k_color_channel[0]);
                    k_color_sampled[1] = in_pixel.getOrderedSample(get_sample, k_color_channel[1]);
                    k_color_sampled[2] = in_pixel.getOrderedSample(get_sample, k_color_channel[2]);
                    k_color_sampled[3] = in_pixel.getOrderedSample(get_sample, k_color_channel[3]);
                    // Clip precision to 4 digits:
                    k_Zf_sampled = rint(k_Zf_sampled * 10000.0)/10000.0;
                    k_Zb_sampled = rint(k_Zb_sampled * 10000.0)/10000.0;
                    k_color_sampled[0] = rint(k_color_sampled[0] * 10000.0)/10000.0;
                    k_color_sampled[1] = rint(k_color_sampled[1] * 10000.0)/10000.0;
                    k_color_sampled[2] = rint(k_color_sampled[2] * 10000.0)/10000.0;
                    k_color_sampled[3] = rint(k_color_sampled[3] * 10000.0)/10000.0;

                    if (k_mode == PATTERN_GET &&
                        (deepInfo.channels().contains(k_spmask_channel[0]) ||
                         deepInfo.channels().contains(k_spmask_channel[1]))) {
                        // Convert floats to SpMask:
                        const float sp1 = in_pixel.getOrderedSample(get_sample, k_spmask_channel[0]);
                        const float sp2 = in_pixel.getOrderedSample(get_sample, k_spmask_channel[1]);
                        m_dpmeta.spmask.fromFloat(sp1, sp2);
                        // Convert to float array and update Array knob:
                        for (int sp_y=0; sp_y < Dcx::SpMask8::height; ++sp_y) {
                            Dcx::SpMask8 sp = (Dcx::SpMask8(0x01) << (sp_y*Dcx::SpMask8::width));
                            for (int sp_x=0; sp_x < Dcx::SpMask8::width; ++sp_x, sp <<= 1)
                                k_spmask_array[sp_x + (Dcx::SpMask8::height-sp_y-1)*Dcx::SpMask8::width] = (sp&m_dpmeta.spmask)?88.0f:0.0f;
                        }
                        sampled_spmask = true;
                    }
                    if (deepInfo.channels().contains(k_flags_channel)) {
                        m_dpmeta.flags = (unsigned)floorf(in_pixel.getOrderedSample(get_sample, k_flags_channel));
                        std::ostringstream oss;
                        m_dpmeta.printFlags(oss);
                        flags_str = oss.str();
                        sampled_flags = true;
                    }
                    sampled_pixel = true;
                }
            }
        }
        //
        Knob* ksamples = knob("num_samples"); assert(ksamples);
        ksamples->set_value(k_num_samples); ksamples->changed();
        //
        Knob* kZf = knob("Zf");
        Knob* kZb = knob("Zb");
        assert(kZf && kZb);
        kZf->set_value(k_Zf_sampled); kZf->changed();
        kZb->set_value(k_Zb_sampled); kZb->changed();
        //
        Knob* kflags = knob("flags"); assert(kflags);
        kflags->set_text(flags_str.c_str()); kflags->changed();
        //
#if 0
        Knob* kcolors = knob("colors"); assert(kcolors);
        kcolors->set_values(k_colors_sampled, 4); kcolors->changed();
#else
        Knob* kcolor = knob("color0"); assert(kcolor);
        kcolor->set_value(k_color_sampled[0]); kcolor->changed();
        kcolor = knob("color1"); assert(kcolor);
        kcolor->set_value(k_color_sampled[1]); kcolor->changed();
        kcolor = knob("color2"); assert(kcolor);
        kcolor->set_value(k_color_sampled[2]); kcolor->changed();
        kcolor = knob("color3"); assert(kcolor);
        kcolor->set_value(k_color_sampled[3]); kcolor->changed();
#endif
        //
        Knob* karray = knob("spmask"); assert(karray);
        karray->set_values(k_spmask_array, Dcx::SpMask8::numBits); karray->changed();
    }

    /*virtual*/
    bool doDeepEngine(Box bbox, const DD::Image::ChannelSet& output_channels, DeepOutputPlane& deep_out_plane) {
        if (!input0())
            return true;

        // We get the values during knob_changed/updateGUI, so we don't do
        // anything special in engine - call the base class:
        if (k_mode == PATTERN_GET)
            return input0()->deepEngine(bbox, output_channels, deep_out_plane);

        // Set the spmask:
        DD::Image::DeepPlane deep_in_plane;
        if (!input0()->deepEngine(bbox, output_channels, deep_in_plane))
            return false;

        deep_out_plane = DD::Image::DeepOutputPlane(output_channels, bbox);

        const int nOutputChans = output_channels.size();

        const int sampleX = int(k_pos[0]);
        const int sampleY = int(k_pos[1]);

        for (Box::iterator it = bbox.begin(); it != bbox.end(); ++it) {
            if (Op::aborted())
                return false; // bail fast on user-interrupt

            // Get the deep pixel from the input:
            if (k_mode == PATTERN_SET && (it.x != sampleX || it.y != sampleY)) {
                deep_out_plane.addPixel(deep_in_plane.getPixel(it));
                continue;
            }

            DD::Image::DeepPixel in_pixel = deep_in_plane.getPixel(it);
            const int nSamples = in_pixel.getSampleCount();

            DD::Image::DeepOutPixel out_pixel;
            out_pixel.reserve(nSamples*nOutputChans);

            for (int i=0; i < nSamples; ++i) {
                if (k_set_all_samples || k_sample == i) {
                    float sp1, sp2;
                    m_dpmeta.spmask.toFloat(sp1, sp2);
                    // Replace spmask channels:
                    foreach (z, output_channels) {
                        if (z == k_spmask_channel[0])
                            out_pixel.push_back(sp1);
                        else if (z == k_spmask_channel[1])
                            out_pixel.push_back(sp2);
#if 0
                        else if (z == k_color_channel[0])
                            out_pixel.push_back();
                        else if (z == k_color_channel[1])
                            out_pixel.push_back();
                        else if (z == k_color_channel[2])
                            out_pixel.push_back();
                        else if (z == k_color_channel[3])
                            out_pixel.push_back();
#endif
                        else
                            out_pixel.push_back(in_pixel.getUnorderedSample(i, z));
                    }

                } else {
                    // Copy all channels:
                    foreach (z, output_channels)
                        out_pixel.push_back(in_pixel.getUnorderedSample(i, z));
                }
            }

            // Add to output plane:
            deep_out_plane.addPixel(out_pixel);
        }

        return true;
    }

};

static Op* build(Node* node) { return new DeepSubpixelMask(node); }
const Op::Description DeepSubpixelMask::description("DeepSubpixelMask", "Deep/DeepSubpixelMask", build);

// end of DeepSubpixelMask.cpp

// TM and (c) 2015-2016 DreamWorks Animation LLC.  All Rights Reserved.
// Reproduction in whole or in part without prior written permission of a
// duly authorized representative is prohibited.
