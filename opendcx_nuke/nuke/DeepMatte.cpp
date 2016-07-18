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
/// @file DeepMatte.cpp

#include "DDImage/DeepFilterOp.h"
#include "DDImage/Knobs.h"

#include <OpenDCX/DcxDeepPixel.h>

using namespace DD::Image;

enum { FLAG_SET, FLAG_CLEAR, FLAG_NOCHANGE };
const char* const flag_modes[] = { "set", "clear", "no-change", 0 };



/*!
*/
class DeepMatte : public DeepFilterOp {
protected:
    int                 k_surface_mode;         //!< How to handle the surface-type (log/lin) flag
    bool                k_fill_spmask;          //!< Fill subpixel mask of non-zero pixels with all ones
    int                 k_matte_mode;           //!< How to handle the matte flag
    bool                k_black_rgb;            //!< Set RGB channels to 0
    int                 k_additive_mode;        //!< How to handle the additive flag
    DD::Image::Channel  k_spmask_channel[2];    //!< Subpixel mask channels
    DD::Image::Channel  k_flags_channel;        //!< Per-sample flags channel
    //
    DD::Image::ChannelSet m_spmask_channels;

public:
    static const Description description;
    /*virtual*/ const char* Class() const { return description.name; }
    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "Modify the matte-object flag on each sample.\n"
        "\n"
        "The matte-object flag is normally stored on the 'spmask.flags' channel which also "
        "contains the log/lin interpolation flag.  The flag bitmask values are:\n"
        "   DEEP_LINEAR_INTERP_SAMPLE = 0x01\n"
        "   DEEP_MATTE_OBJECT_SAMPLE  = 0x02\n"
        "   DEEP_ADDITIVE_SAMPLE      = 0x04\n"
        "\n"
        "When deep pixels are flattened (DeepToImage, DeepCamDefocus) the matte-object "
        "flag is used to make sure the alpha channel is properly cutout.";
    }

    DeepMatte(Node* node) : DeepFilterOp(node) {
        k_spmask_channel[0] = DD::Image::getChannel(Dcx::spMask8ChannelName1);
        k_spmask_channel[1] = DD::Image::getChannel(Dcx::spMask8ChannelName2);
        k_flags_channel     = DD::Image::getChannel(Dcx::flagsChannelName);
        k_surface_mode      = FLAG_NOCHANGE;
        k_fill_spmask       = false;
        k_matte_mode        = FLAG_SET;
        k_black_rgb         = true;
        k_additive_mode     = FLAG_NOCHANGE;
    }

    /*virtual*/ Op* op() { return this; }

    /*virtual*/
    void knobs(Knob_Callback f) {
        deep_surface_knobs(f, false/*show_surface_knobs*/, true/*show_matte_knobs*/);
    }

    void deep_surface_knobs(Knob_Callback f, bool show_surface_knobs, bool show_matte_knobs) {
        if (show_surface_knobs) {
            Enumeration_knob(f, &k_surface_mode, flag_modes, "hard_surface_mode", "hard-surface flag");
                Tooltip(f, "Set/clear the hard-surface flag.  If it's on the samples are interpreted as "
                           "hard-surfaces with linear interpolation and if off the sample is volumetric "
                           "with log interpolation.");
            Bool_knob(f, &k_fill_spmask, "fill_spmask", "fill spmask");
                Tooltip(f, "Fill subpixel-mask of non-zero pixels with all ones (turn on all bits.)");
        }
        if (show_matte_knobs) {
            Newline(f);
            Enumeration_knob(f, &k_matte_mode, flag_modes, "matte_mode", "matte-cutout flag");
                Tooltip(f, "Set/clear the matte-cutout flag.   (see node help for more info)");
            Bool_knob(f, &k_black_rgb, "black_rgb", "set rgb channels to black");
                Tooltip(f, "Set RGB color channels to black/zero.  At the moment this does not affect "
                           "color AOVs, only RGB.");
        }
        if (show_surface_knobs) {
            Newline(f);
            Enumeration_knob(f, &k_additive_mode, flag_modes, "additive_mode", "additive flag");
                Tooltip(f, "Set/clear the additive flag.  If it's on the samples are interpreted as "
                           "having alpha/coverage and/or cutouts already applied and should be added to "
                           "each other rather than merged.\n"
                           "\n"
                           "ex. mutually-cutout objects DeepMerged together will produces cracks along "
                           "shared edges due to double-premulting during the flatten operation. Setting "
                           "the samples to additive mode (either before or after the DeepMerge) stops "
                           "the double-premult from happening.");
        }
        Input_Channel_knob(f, k_spmask_channel, 2/*nChans*/, 0/*input*/, "spmask_channels", "spmask channels");
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channels which contain the per-sample spmask data. Two channels are required for an 8x8 mask.");
        Input_Channel_knob(f, &k_flags_channel, 1/*nChans*/, 0/*input*/, "flags_channel", "flags");
            ClearFlags(f, Knob::STARTLINE);
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channel which contains the per-sample flag data.");
    }

    /*virtual*/ 
    void _validate(bool for_real) {
        DeepFilterOp::_validate(for_real);

        // Get subpixel mask channels:
        m_spmask_channels = DD::Image::Mask_None;
        if (k_fill_spmask) {
            m_spmask_channels += k_spmask_channel[0];
            m_spmask_channels += k_spmask_channel[1];
        }
        m_spmask_channels += k_flags_channel;
        // Set output channels:
        DD::Image::ChannelSet output_channels = _deepInfo.channels();
        if (k_surface_mode != FLAG_NOCHANGE || k_matte_mode != FLAG_NOCHANGE)
            output_channels += m_spmask_channels;
        _deepInfo = DeepInfo(_deepInfo.formats(), _deepInfo.box(), output_channels);
    }

    /*virtual*/
    void getDeepRequests(Box bbox, const DD::Image::ChannelSet& channels, int count, std::vector<RequestData>& requests) {
        if (!input0())
            return;
        DD::Image::ChannelSet get_channels = channels;
        if (k_surface_mode != FLAG_NOCHANGE || k_matte_mode != FLAG_NOCHANGE)
            get_channels += m_spmask_channels;
        requests.push_back(RequestData(input0(), bbox, get_channels, count));
    }

    /*virtual*/
    bool doDeepEngine(Box bbox, const DD::Image::ChannelSet& output_channels, DeepOutputPlane& deep_out_plane) {
        if (!input0())
            return true;

        deep_out_plane = DeepOutputPlane(output_channels, bbox);

        DD::Image::ChannelSet get_channels = output_channels;
        if (k_surface_mode != FLAG_NOCHANGE || k_matte_mode != FLAG_NOCHANGE)
           get_channels += m_spmask_channels;

        DeepPlane deep_in_plane;
        if (!input0()->deepEngine(bbox, get_channels, deep_in_plane))
            return false;

        const DD::Image::ChannelSet input_channels = input0()->deepInfo().channels();

        // Get the set of color channels to blacken:
        DD::Image::ChannelSet black_channels(DD::Image::Mask_None);
        if (k_black_rgb) {
            black_channels = DD::Image::Mask_RGB; // Only do RGB for now
            // TODO: extend to find valid RGB channels in all layers that have at least 3 channels
        }

        const bool modify_flags = (k_surface_mode  != FLAG_NOCHANGE ||
                                   k_matte_mode    != FLAG_NOCHANGE ||
                                   k_additive_mode != FLAG_NOCHANGE);
        const bool no_changes = (!modify_flags &&
                                 !k_black_rgb &&
                                 !k_fill_spmask);

        float sp1_full, sp2_full;
        Dcx::SpMask8::fullCoverage.toFloat(sp1_full, sp2_full);

        const int nOutputChans = output_channels.size();
        for (Box::iterator it = bbox.begin(); it != bbox.end(); ++it) {
            if (Op::aborted())
                return false; // bail fast on user-interrupt

            // Get the deep pixel from the input plane:
            DeepPixel in_pixel = deep_in_plane.getPixel(it);
            const unsigned nSamples = in_pixel.getSampleCount();
            if (nSamples == 0) {
                deep_out_plane.addHole(); // no samples, skip it
                continue;
            }

            DeepOutPixel out_pixel;
            out_pixel.reserve(nSamples*nOutputChans);

            if (no_changes) {
                for (unsigned i=0; i < nSamples; ++i) {
                    foreach(z, output_channels) {
                        if (input_channels.contains(z))
                            out_pixel.push_back(in_pixel.getUnorderedSample(i, z));
                        else
                            out_pixel.push_back(0.0f);
                    }
                }
                // Add to output plane:
                deep_out_plane.addPixel(out_pixel);
                continue;
            }

            // Change samples:
            for (unsigned i=0; i < nSamples; ++i) {
                foreach(z, output_channels) {

                    if ((z == k_spmask_channel[0] || z == k_spmask_channel[1]) &&
                            k_surface_mode != FLAG_NOCHANGE && k_fill_spmask) {
                        // Fill the spmask channels with ones:
                        out_pixel.push_back(sp1_full); // both channels are same bit pattern

                    } else if (z == k_flags_channel && modify_flags) {
                        // Modify the flags:
                        Dcx::DeepFlag flags = Dcx::DEEP_EMPTY_FLAG;
                        if (input_channels.contains(k_flags_channel))
                            flags = (int)floorf(in_pixel.getUnorderedSample(i, z));
                        //
                        if (k_surface_mode == FLAG_SET)
                            flags |= Dcx::DEEP_LINEAR_INTERP_SAMPLE;
                        else if (k_surface_mode == FLAG_CLEAR)
                            flags &= ~Dcx::DEEP_LINEAR_INTERP_SAMPLE;
                        if (k_matte_mode == FLAG_SET)
                            flags |= Dcx::DEEP_MATTE_OBJECT_SAMPLE;
                        else if (k_matte_mode == FLAG_CLEAR)
                            flags &= ~Dcx::DEEP_MATTE_OBJECT_SAMPLE;
                        if (k_additive_mode == FLAG_SET)
                            flags |= Dcx::DEEP_ADDITIVE_SAMPLE;
                        else if (k_additive_mode == FLAG_CLEAR)
                            flags &= ~Dcx::DEEP_ADDITIVE_SAMPLE;
                        //
                        out_pixel.push_back(float(flags));

                    } else if (black_channels.contains(z)) {
                        // Blacken channel:
                        out_pixel.push_back(0.0f);

                    } else {
                        // Copy unchanged:
                        if (input_channels.contains(z))
                            out_pixel.push_back(in_pixel.getUnorderedSample(i, z));
                        else
                            out_pixel.push_back(0.0f);
                    }
                }
            }
            // Add to output plane:
            deep_out_plane.addPixel(out_pixel);
        }

        return true;
    }

};

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

/*! Variant of DeepMatte that exposes all the surface knobs.
*/
class DeepSurfaceType : public DeepMatte {
public:
    static const Description description_surface;
    ///*virtual*/ const char* Class() const { return description_surface.name; }
    /*virtual*/ const char* displayName() const { return "DeepSurfaceType"; }
    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "Modify the deep sample flags on each sample.\n"
        "\n"
        "The surface-type, matte-object and additive-mode flags are stored on the "
        "'spmask.flags' channel. The flag bitmask values are:\n"
        "   DEEP_LINEAR_INTERP_SAMPLE = 0x01\n"
        "   DEEP_MATTE_OBJECT_SAMPLE  = 0x02\n"
        "   DEEP_ADDITIVE_SAMPLE      = 0x04\n"
        "\n"
        "When deep pixels are flattened (DeepToImage, DeepCamDefocus) the flags "
        "are used to modify the merging math per-sample.";
    }

    DeepSurfaceType(Node* node) : DeepMatte(node) {
        k_surface_mode  = FLAG_NOCHANGE;
        k_fill_spmask   = false;
        k_matte_mode    = FLAG_NOCHANGE;
        k_black_rgb     = false;
        k_additive_mode = FLAG_NOCHANGE;
    }

    /*virtual*/
    void knobs(Knob_Callback f) {
        deep_surface_knobs(f, true/*show_surface_knobs*/, true/*show_matte_knobs*/);
    }
};

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

// Build a DeepMatte:
static Op* build(Node* node) { return new DeepMatte(node); }
const Op::Description DeepMatte::description("DeepMatte", "Deep/DeepMatte", build);

// Build a DeepSurfaceType:
static Op* build_surface(Node* node) { return new DeepSurfaceType(node); }
const Op::Description DeepSurfaceType::description_surface("DeepSurfaceType", "Deep/DeepSurfaceType", build_surface);
