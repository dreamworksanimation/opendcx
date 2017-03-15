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
/// @file DeepToImage.cpp


#include <DDImage/Iop.h>
#include <DDImage/Row.h>
#include <DDImage/DeepOp.h>
#include <DDImage/Knobs.h>
#include <DDImage/Pixel.h>
#include <DDImage/Filter.h>
#include <DDImage/DeepComposite.h>

#include <OpenDCX/DcxDeepPixel.h>

#include "DDImageAdapter.h"


//#define DCX_DEBUG_FLATTENER 1
#ifdef DCX_DEBUG_FLATTENER
static bool show_debug = true;
#else
static bool show_debug = false;
#endif

using namespace DD::Image;

//
//  DeepToImage
//
//      A Nuke plugin to flatten deep pixel samples into a 2D flat image with subpixel
//      mask and varying surface type support.
//
//      This is a customized version of the stock Foundry node and provides support
//      for subpixel sampling via subpixel-mask channels (spmask layer by default).
//      Note: If no subpixel mask channels exist on the input then the standard flattening
//      (legacy) behavior is performed, providing backwards-compatibility for older deep
//      images.
//
//      If non-spmask images are merged with spmask images then the non-spmask samples
//      will have spmask channels containing zeros (0). Zero spmask values are
//      interpreted by the flattener as a legacy sample and a full-coverage mask is then
//      used, with the possible result of some aliasing at pixel edges.
//
//      **********************************************************************************
//      **********************************************************************************
//      **                                                                              **
//      ** Nuke transparently inserts a DeepToImage node into the Viewer's input stream **
//      ** when it's connected directly to a Deep node.                                 **
//      **                                                                              **
//      ** So - this plugin MUST override the stock Foundry DeepToImage otherwise the   **
//      ** spmask/flag data will be ignored for deep samples that contain it, likely    **
//      ** producing aliased and unpremulted images.                                    **
//      **                                                                              **
//      **********************************************************************************
//      **********************************************************************************
//
//
//  TODO:
//    * Add pixel-filtering support back in. This was removed for the first public release
//      as it implemented internal DreamWorks filters. A pixel filter handler class should
//      be added to the Dcx lib with a set of common pixel filters that are not present in
//      DDImage, like Blackman-Harris.
//
//    * Make sure DDImageAdpater dcxChannelSet() & dcxChannel() calls are thread safe,
//      as they're being called from engine().
//
//    * Fix or work around gcc crash writing to channel 12
//
//    * Remove debug switches
//

//----------------------------------------------------------------------------------------

// Keep this matching Dcx::DeepPixel::InterpolationMode enums:
static const char* interpolation_modes[] = { "off", "auto", "log", "lin", 0 };


/*! Example Deep flattener Nuke plugin which supports subpixel masks and
    sample flags.
*/
class DeepToImage : public DD::Image::Iop {
    bool                    k_one_over_z;           //!< Output 1/Z or Z?
    bool                    k_cutout_z;             //!< Does a cutout operation affect Z?
    bool                    k_cutout_color;         //!< Does a cutout operation affect color channels?
    //
    int                     k_interpolation_mode;   //!< Interpolation mode
    bool                    k_use_spmasks;          //!< Use subpixel masks
    bool                    k_use_flags;            //!< Use deep flags
    DD::Image::Channel      k_spmask_channel[2];    //!< Per-sample subpixel mask channels
    DD::Image::Channel      k_flags_channel;        //!< Per-sample flags channel
    //
    float                   k_debug[2];
    int                     k_debug_sp_x, k_debug_sp_y;
    //
    DD::Image::ChannelSet   m_input_spmask_channels;
    Box                     m_request_bbox;         //!< Remember the requested bbox

public:                                  
    static const Description description;
    /*virtual*/ const char* Class() const { return description.name; }

    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "Flattens deep pixel samples into a 2D 'flat' image with subpixel mask and "
        "pixel filtering support.\n"
        "\n"
        "This is a customized version of the stock Foundry node and provides support "
        "for subpixel sampling via subpixel-mask channels (spmask layer by default).\n"
        "Note: If no subpixel mask channels exist on the input then the standard flattening "
        "(legacy) behavior is performed, providing backwards-compatibility for older deep "
        "images.\n"
        "\n"
        "If non-spmask images are merged with spmask images then the non-spmask samples "
        "will have spmask channels containing zeros (0). Zero spmask values are "
        "interpreted by the flattener as a legacy sample and a full-coverage mask is then "
        "used, with the possible result of some aliasing at pixel edges.";
    }

    /*virtual*/ const char* node_shape() const { return DeepOp::DeepNodeShape(); }

    DeepToImage(Node* node) : Iop(node) {
        // Get OpenDCX standard channels assigned in the correct order:
        Dcx::dcxGetSpmaskChannels(k_spmask_channel[0], k_spmask_channel[1], k_flags_channel);
        //
        k_one_over_z         = true;
        k_cutout_z           = true;
        k_cutout_color       = true;
        //
        k_interpolation_mode = (int)Dcx::DeepPixel::INTERP_AUTO;
        k_use_spmasks        = true;
        k_use_flags          = true;
        //
        k_debug[0] = k_debug[1] = -1.0f; // no debug
        k_debug_sp_x = k_debug_sp_y = -1;

        m_input_spmask_channels = DD::Image::Mask_None;
    }

    /*virtual*/ Op*  default_input(int input) const { return 0; } // allow null inputs
    /*virtual*/ int  minimum_inputs() const { return 1; }
    /*virtual*/ int  maximum_inputs() const { return 1; }
    /*virtual*/ bool test_input(int input, Op*op) const { return dynamic_cast<DeepOp*>(op); }

    DeepOp* input0() { return dynamic_cast<DeepOp*>(Op::input(0)); }

    /*virtual*/
    void knobs(Knob_Callback f) {
        Obsolete_knob(f, "volumetric_composition", 0); // Avoids knob errors loading stock Foundry DeepToImage node

        Bool_knob(f, &k_one_over_z, "one_over_z", "output 1/z");
            ClearFlags(f, Knob::STARTLINE);
            Tooltip(f, "Output Nuke-style 1/Z in <b>depth.Z</b> where maxZ=0 & minZ=1.0, and Z *decreases* with distance.\n"
                       "If disabled, output distance from camera where maxZ=infinity and Z *increases* with distance.\n"
                       "\n"
                       "Note: <b>deep.front</b> & <b>deep.back</b> are unaffected by this switch and remain absolute Z");
        //Bool_knob(f, &k_cutout_z, "cutout_z", "cutout z");
        //    Tooltip(f, "If on matte-cutout operations also cutout Z channels (Z, DeepFront, DeepBack.)  If off then "
        //               "the resulting Z values include the cutout surfaces.");
        //Bool_knob(f, &k_cutout_color, "cutout_color", "cutout color");
        Newline(f);
        Enumeration_knob(f, &k_interpolation_mode, interpolation_modes, "interpolation_mode", "interpolation");
            Tooltip(f, "<b>auto</b> mode uses the spmask flags channel to identify the per-sample interpolation "
                       "mode allowing the merging of lin/lin, lin/log & log/log sample combinations to be "
                       "handled correctly.\n"
                       "\n"
                       "<b>log</b> and <b>lin</b> forces the interpolation mode, ignoring the per-sample info.\n"
                       "\n"
                       "<b>off</b> disables overlap interpolation entirely so flattening assumes all samples "
                       "are separated in Z. If any samples *do* overlap this mode will likely produce artifacts "
                       "(for debugging purposes only)");
        Bool_knob(f, &k_use_spmasks, "use_subpixel_masks", "use subpixel masks");
            Tooltip(f, "If enabled and the specified subpixel mask channels are present on the input "
                       "then build a subpixel mask from them to perform individual subpixel intersection "
                       "testing.\n"
                       "This produces more accurate flattening results, especially for overlapping "
                       "hard-surface renders.");
        Obsolete_knob(f, "subpixel_mask_mode", "if {$value=={off}} {knob use_subpixel_masks false}");
        Bool_knob(f, &k_use_flags, "use_flags_channel", "use deep flags");
            Tooltip(f, "If enabled then the flattener uses the deep sample flags information stored in the flags channel.");
        Newline(f);
        //
        Newline(f);
        Input_Channel_knob(f, k_spmask_channel, 2/*nChans*/, 0/*input*/, "spmask_channels", "spmask channels");
            Tooltip(f, "Channels which contain the per-sample spmask data. Two channels are required for an 8x8 mask.");
        Input_Channel_knob(f, &k_flags_channel, 1/*nChans*/, 0/*input*/, "flags_channel", "flags");
            ClearFlags(f, Knob::STARTLINE);
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channel which contains the per-sample flag data.");
        //
        if (show_debug)
            Divider(f);
        XY_knob(f, k_debug, "debug", (show_debug)?"debug":INVISIBLE);
        Int_knob(f, &k_debug_sp_x, "debug_sp_x", (show_debug)?" subpixel":INVISIBLE);
            SetFlags(f, Knob::NO_ANIMATION | Knob::NO_MULTIVIEW);
            ClearFlags(f, Knob::STARTLINE);
        Int_knob(f, &k_debug_sp_y, "debug_sp_y", (show_debug)?"":INVISIBLE);
            SetFlags(f, Knob::NO_ANIMATION | Knob::NO_MULTIVIEW);
            ClearFlags(f, Knob::STARTLINE);
    }

   /*virtual*/
   int knob_changed(Knob* k) {
      if (k == &Knob::showPanel ||
          k->name() == "use_flags_channel" || k->name() == "use_subpixel_masks") {
         knob("spmask_channels")->enable(k_use_spmasks);
         knob("flags_channel")->enable(k_use_flags);
         return 1;
      }
      return Iop::knob_changed(k);
   }

    // Modify the input channels that the flattener needs from its input.
    void getInputChannelSet(DD::Image::ChannelSet& channels) const {
        channels += DD::Image::Mask_Alpha;
        channels += DD::Image::Mask_DeepFront;
        if (k_interpolation_mode != (int)Dcx::DeepPixel::INTERP_OFF)
            channels += DD::Image::Mask_DeepBack;
        channels += k_spmask_channel[0];
        channels += k_spmask_channel[1];
        if (k_use_flags)
            channels += k_flags_channel;
        channels -= DD::Image::Mask_Z;
    }

    /*virtual*/
    void _validate(bool for_real) {
        m_input_spmask_channels = DD::Image::Mask_None;

        // Input may be null, don't crash:
        if (!input0()) {
            info_.set(DD::Image::Box());
            info_.channels(DD::Image::Mask_None);

        } else {
            input0()->validate(true);
            DeepInfo deepInfo = input0()->deepInfo();
#ifdef DCX_DEBUG_FLATTENER
            std::cout << "DeepToImage(" << this << ")::_validate(" << for_real << "): channels=" << deepInfo.channels() << std::endl;
#endif

            DD::Image::ChannelSet out_channels = deepInfo.channels();
            // Always output depths:
            out_channels += DD::Image::Mask_Z;
            out_channels += DD::Image::Mask_Deep;

            // Get spmask params from input channels - need both channels for an 8x8 mask!
            if (k_use_spmasks &&
                    k_spmask_channel[0] != DD::Image::Chan_Black && deepInfo.channels().contains(k_spmask_channel[0]) &&
                    k_spmask_channel[1] != DD::Image::Chan_Black && deepInfo.channels().contains(k_spmask_channel[1])) {
                m_input_spmask_channels += k_spmask_channel[0];
                m_input_spmask_channels += k_spmask_channel[1];
            }

            // Don't output the subpixel mask or flags channels:
            out_channels -= m_input_spmask_channels;
            if (k_use_flags)
                out_channels -= k_flags_channel;

            // Copy some info from input DeepOp:
            info_.setFormats(deepInfo.formats());
            info_.first_frame(deepInfo.firstFrame());
            info_.last_frame(deepInfo.lastFrame());  
            info_.set(deepInfo.box());
            info_.channels(out_channels);
            set_out_channels(out_channels); // is this still needed...?

            input0()->op()->cached(cached());
        }
    }

    /*virtual*/
    void _request(int x, int y, int r, int t, ChannelMask in_channels, int count) {                           
        if (!input0())
            return; // don't crash...
        DD::Image::ChannelSet get_channels = in_channels;
        getInputChannelSet(get_channels);
        input0()->deepRequest(DD::Image::Box(x, y, r, t), get_channels, count);
    }

    /*! Flatten a row of deep pixels with support for subpixel masks and sample metadata.
    */
    /*virtual*/
    void engine(int y, int x, int r, ChannelMask out_channels, Row& out_row) {
        //std::cout << y << " DeepToImage::engine() out_channels=" << out_channels << ", x=" << x << ", r=" << r << std::endl;
        if (!input0()) {
            // Don't crash...
            foreach(z, out_channels)
                out_row.erase(z);
            return;
        }

        DD::Image::ChannelSet get_channels = out_channels;
        getInputChannelSet(get_channels);

        DD::Image::DeepPlane in_deep_plane;
        if (!input0()->deepEngine(y, x, r, get_channels, in_deep_plane)) {
            Iop::abort();
            foreach(z, out_channels)
                out_row.erase(z);
            return;
        }

        DD::Image::ChannelSet out_z_channels(DD::Image::Mask_Z | DD::Image::Mask_Deep);
        out_z_channels &= out_channels;
        const bool write_z = (out_channels.contains(DD::Image::Chan_Z));

        DD::Image::ChannelSet flatten_color_channels = out_channels;
        flatten_color_channels += DD::Image::Chan_Alpha; // always need alpha!
        flatten_color_channels -= DD::Image::Mask_Deep;
        flatten_color_channels -= DD::Image::Mask_Z;
        flatten_color_channels -= m_input_spmask_channels;
        if (k_use_flags)
            flatten_color_channels -= k_flags_channel;

        // Don't use Row::erase(),we want to fill the row memory:
        foreach(z, out_channels)
            memset(out_row.writable(z)+x, 0, (r-x)*sizeof(float));
        foreach(z, out_z_channels) {
            float* OUT = out_row.writable(z)+x;
            for (int X=x; X < r; ++X)
                *OUT++ = INFINITY;
        }

        // Set up OpenDCX objects:
        Dcx::ChannelSet dcx_flatten_channels(Dcx::dcxChannelSet(flatten_color_channels));
        Dcx::ChannelSet dcx_out_channels(Dcx::dcxChannelSet(out_channels));
        Dcx::DeepPixel  dcx_deep_pixel(dcx_flatten_channels);
        Dcx::Pixelf     dcx_flattened(dcx_out_channels);

        // DeepPixel::flatten() tests for legacy deep pixels vs ones with spmasks:
        for (int X=x; X < r; ++X) {
#ifdef DCX_DEBUG_FLATTENER
            const bool debug = (X == int(k_debug[0]) && y == int(k_debug[1]));
            if (debug) {
               std::cout << "DeepToImage(" << this << ")::flatten pixel [" << X << " " << y << "]";
               //std::cout << ", k_flags_channel=" << k_flags_channel << ", k_use_flags=" << k_use_flags;
               std::cout << std::endl;
            }
#endif
            Dcx::dcxCopyDDImageDeepPixel(in_deep_plane.getPixel(y, X),
                                         (k_use_spmasks)?k_spmask_channel[0]:DD::Image::Chan_Black,
                                         (k_use_spmasks)?k_spmask_channel[1]:DD::Image::Chan_Black,
                                         (k_use_flags)?k_flags_channel:DD::Image::Chan_Black,
                                         dcx_deep_pixel);
            if (dcx_deep_pixel.size() == 0)
                continue; // Skip if nothing to flatten

            dcx_deep_pixel.setXY(X, y); // xy coord in DeepPixel is typically used just for debugging
#ifdef DCX_DEBUG_FLATTENER
            if (debug) {
                dcx_deep_pixel.setXY(-100000, -100000); // enable debug prints from OpenDCX debug
                //dcx_deep_pixel.printInfo(std::cout, "  in pixel"/*prefix*/, 4/*padding*/, true/*show_mask*/);

                // If debug subpixel selected only flatten it to isolate its contribution:
                if (k_debug_sp_x >= 0 && k_debug_sp_x < Dcx::SpMask8::width &&
                    k_debug_sp_y >= 0 && k_debug_sp_y < Dcx::SpMask8::height) {
                    std::cout << "  DeepToImage::flatten subpixel [" << k_debug_sp_x << " " << k_debug_sp_y << "]" << std::endl;
                    dcx_deep_pixel.flattenSubpixels(dcx_flatten_channels,
                                                    dcx_flattened,
                                                    Dcx::SpMask8::getMask(k_debug_sp_x, k_debug_sp_y),
                                                    (Dcx::DeepPixel::InterpolationMode)k_interpolation_mode);
                } else {
                    dcx_deep_pixel.flatten(dcx_flatten_channels,
                                           dcx_flattened,
                                           (Dcx::DeepPixel::InterpolationMode)k_interpolation_mode);
                }
            } else {
                dcx_deep_pixel.flatten(dcx_flatten_channels,
                                       dcx_flattened,
                                       (Dcx::DeepPixel::InterpolationMode)k_interpolation_mode);
            }
#else
            dcx_deep_pixel.flatten(dcx_flatten_channels,
                                   dcx_flattened,
                                   (Dcx::DeepPixel::InterpolationMode)k_interpolation_mode);
#endif
            // Copy flattened pixel to output row:
            foreach(z, out_channels)
                out_row.writable(z)[X] = dcx_flattened[Dcx::dcxChannel(z)];
            if (write_z)
                out_row.writable(DD::Image::Chan_Z)[X] = dcx_flattened[Dcx::Chan_ZFront];
        }

        // Apply 1/Z now if desired, but only to DD::Image::Chan_Z:
        if (write_z && k_one_over_z) {
            float* OUTZ = out_row.writable(DD::Image::Chan_Z) + x;
            for (int X=x; X < r; ++X) {
                if (isnan(*OUTZ) || isinf(*OUTZ))
                    *OUTZ = 0.0f;
                else if (*OUTZ > EPSILONf)
                    *OUTZ = 1.0f / *OUTZ;
                else
                    *OUTZ = 0.0f;
                ++OUTZ;
            }
        }

    } // engine()

};

static Op* build(Node* node) { return new DeepToImage(node); }
const Op::Description DeepToImage::description("DeepToImage", build);
