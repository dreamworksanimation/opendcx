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

using namespace DD::Image;


//----------------------------------------------------------------------------------------

// Keep these matching Dcx::SpMaskMode and Dcx::InterpolationMode enums
static const char* spmask_modes[]        = { "off", "auto", "4x4", "8x8", /*"16x16",*/ 0 }; // TODO: DEPRECATE! CHANGE TO ON/OFF BOOL
static const char* interpolation_modes[] = { "off", "auto", "log", "lin", 0 };


/*! Example Deep flattener Nuke plugin which supports subpixel masks and
    sample flags.
*/
class DeepToImage : public DD::Image::Iop {
    bool                    k_one_over_z;           //!< Output 1/Z or Z
    bool                    k_cutout_z;             //!< Does a cutout operation affect Z?
    bool                    k_cutout_color;         //!< Does a cutout operation affect color channels?
    //
    int                     k_interpolation_mode;   //!< Interpolation mode
    int                     k_spmask_mode;          //!< Subpixel mask mode - TODO: DEPRECATE! CHANGE TO ON/OFF BOOL
    DD::Image::Channel      k_spmask_channel[2];    //!< Subpixel mask channels
    DD::Image::Channel      k_flags_channel;        //!< Per-sample flags channel
#ifdef DCX_DEBUG_FLATTENER
    float                   k_debug[2];
    int                     k_debug_sp_x, k_debug_sp_y;
#endif


    DD::Image::ChannelSet   m_spmask_channels;      //!< 
    int                     m_spmask_size;          //!< 0, 4, or 8 - TODO: DEPRECATE! CHANGE TO ON/OFF BOOL
    bool                    m_spmask_have_flags;    //!< Are flags active?
    Box                     m_request_bbox;         //!< Remember the requested bbox

public:                                  
    static const Description description;
    /*virtual*/ const char* Class() const { return description.name; }

    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "Flattens deep pixel samples into a 2D 'flat' image with subpixel mask and "
        "pixel filtering support.\n"
        "\n"
        "This is a customized version of the stock Foundry node and provides support "
        "for subpixel sampling via subpixel-mask channels (spmask) of either 4x4(16-bits) "
        "or 8x8(64-bits) sizes.  Pixel-filtering is supported when subpixel masks are "
        "available.\n"
        "Note: If no subpixel mask channels exist on the input then the standard flattening "
        "(legacy) behavior is performed, providing backwards-compatibility for older deep "
        "images.\n"
        "\n"
        "If non-spmask images are merged with spmask images then the non-spmask samples "
        "will have spmask channels added containing zeros (0).  Zero spmask values are "
        "interpreted by the flattener as a legacy sample and a full-coverage mask is then "
        "used, with the possible result of some aliasing at pixel edges.";
    }

    /*virtual*/ const char* node_shape() const { return DeepOp::DeepNodeShape(); }

    DeepToImage(Node* node) : Iop(node) {
        k_one_over_z         = true;
        k_cutout_z           = true;
        k_cutout_color       = true;
        //
        k_interpolation_mode = (int)Dcx::INTERP_AUTO;
        k_spmask_mode        = (int)Dcx::SPMASK_AUTO; // TODO: DEPRECATE! CHANGE TO ON/OFF BOOL
        k_spmask_channel[0]  = DD::Image::getChannel(Dcx::spMask8ChannelName1);
        k_spmask_channel[1]  = DD::Image::getChannel(Dcx::spMask8ChannelName2);
        k_flags_channel      = DD::Image::getChannel(Dcx::flagsChannelName);
#ifdef DCX_DEBUG_FLATTENER
        k_debug[0] = k_debug[1] =-1.0f; // no debug
        k_debug_sp_x = k_debug_sp_y = -1;
#endif

        m_spmask_channels    = DD::Image::Mask_None;
        m_spmask_size        = 0;
        m_spmask_have_flags  = false;
    }

    /*virtual*/ Op*  default_input(int input) const { return 0; } // allow null inputs
    /*virtual*/ int  minimum_inputs() const { return 1; }
    /*virtual*/ int  maximum_inputs() const { return 1; }
    /*virtual*/ bool test_input(int input, Op*op) const { return dynamic_cast<DeepOp*>(op); }

    DeepOp* input0() { return dynamic_cast<DeepOp*>(Op::input(0)); }

    /*virtual*/
    void knobs(Knob_Callback f) {
        // Avoids knob errors loading stock Foundry DeepToImage node:
        Obsolete_knob(f, "volumetric_composition", 0);

        Bool_knob(f, &k_one_over_z, "one_over_z", "output 1/z");
            ClearFlags(f, Knob::STARTLINE);
            Tooltip(f, "Output Nuke-style 1/Z (depth.Z) where maxZ=0 & minZ=1.0, and Z *decreases* with distance.\n"
                       "If disabled, output 'straight' depth.Z where maxZ=infinity and Z *increases* with distance.\n"
                       "\n"
                       "deep.front & deep.back are unaffected by this switch and remain absolute Z");
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
        Newline(f);
        // TODO: DEPRECATE! CHANGE TO ON/OFF BOOL:
        Enumeration_knob(f, &k_spmask_mode, spmask_modes, "subpixel_mask_mode", "subpixel mask");
            Tooltip(f, "If <b>auto</b> and the specified subpixel mask channels are present on the input "
                       "then build a subpixel mask from them to perform individual subpixel intersection "
                       "testing. This produces more accurate results, especially with overlapping "
                       "hard-surface renders.");
        Input_Channel_knob(f, k_spmask_channel, 2/*nChans*/, 0/*input*/, "spmask_channels", "spmask channels");
            Tooltip(f, "Channels which contain the per-sample spmask data. Two channels are required for an 8x8 mask.");
        Input_Channel_knob(f, &k_flags_channel, 1/*nChans*/, 0/*input*/, "flags_channel", "flags");
            ClearFlags(f, Knob::STARTLINE);
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channel which contains the per-sample flag data.");
#ifdef DCX_DEBUG_FLATTENER
        Divider(f);
        XY_knob(f, k_debug, "debug", "debug");
        Int_knob(f, &k_debug_sp_x, "debug_sp_x", " subpixel");
            SetFlags(f, Knob::NO_ANIMATION | Knob::NO_MULTIVIEW);
            ClearFlags(f, Knob::STARTLINE);
        Int_knob(f, &k_debug_sp_y, "debug_sp_y", "");
            SetFlags(f, Knob::NO_ANIMATION | Knob::NO_MULTIVIEW);
            ClearFlags(f, Knob::STARTLINE);
#else
        Obsolete_knob(f, "debug", 0);
        Obsolete_knob(f, "debug_sp_x", 0);
        Obsolete_knob(f, "debug_sp_y", 0);
#endif
    }

    // Modify the input channels that the flattener needs from its input.
    void getInputChannelSet(DD::Image::ChannelSet& channels) const {
        channels += DD::Image::Mask_Alpha;
        channels += DD::Image::Mask_DeepFront;
        if (k_interpolation_mode != (int)Dcx::INTERP_OFF)
            channels += DD::Image::Mask_DeepBack;
        channels += k_spmask_channel[0];
        channels += k_spmask_channel[1];
        channels += k_flags_channel;
        channels -= DD::Image::Mask_Z;
    }

    /*virtual*/
    void _validate(bool for_real) {
        m_spmask_channels   = DD::Image::Mask_None;
        m_spmask_size       = 0;
        m_spmask_have_flags = false;

        // Input may be null, don't crash:
        if (!input0()) {
            info_.set(DD::Image::Box());
            info_.channels(DD::Image::Mask_None);

        } else {
            input0()->validate(true);
            DeepInfo deepInfo = input0()->deepInfo();

            DD::Image::ChannelSet out_channels = deepInfo.channels();
            out_channels += DD::Image::Mask_Z;
            out_channels += DD::Image::Mask_Deep;

            // Get spmask params from input channels:
            // TODO: DEPRECATE! CHANGE TO ON/OFF BOOL
            if (k_spmask_mode != (int)Dcx::SPMASK_OFF) {
                if (k_spmask_mode == (int)Dcx::SPMASK_AUTO) {
                    if (k_spmask_channel[0] != DD::Image::Chan_Black &&
                          deepInfo.channels().contains(k_spmask_channel[0])) {
                       m_spmask_channels += k_spmask_channel[0];
                       m_spmask_size += 4;
                    }
                    if (k_spmask_channel[1] != DD::Image::Chan_Black &&
                          deepInfo.channels().contains(k_spmask_channel[1])) {
                       m_spmask_channels += k_spmask_channel[1];
                       m_spmask_size += 4;
                    }

                } else if (k_spmask_mode == (int)Dcx::SPMASK_4x4 &&
                    k_spmask_channel[0] != DD::Image::Chan_Black) {
                    m_spmask_channels += k_spmask_channel[0];
                    m_spmask_size = 4;

                } else if (k_spmask_mode == (int)Dcx::SPMASK_8x8 &&
                           k_spmask_channel[0] != DD::Image::Chan_Black &&
                           k_spmask_channel[1] != DD::Image::Chan_Black) {
                    m_spmask_channels += k_spmask_channel[0];
                    m_spmask_channels += k_spmask_channel[1];
                    m_spmask_size = 8;

                } else {
                    // unsupported spmask mode
                }

                // If we have a third input spmask channel it's reserved for flags:
                if (k_flags_channel != DD::Image::Chan_Black &&
                       deepInfo.channels().contains(k_flags_channel)) {
                    m_spmask_have_flags = true;
                    m_spmask_channels += k_flags_channel;
                }
            }

            // Don't output the subpixel mask channels:
            out_channels -= m_spmask_channels;

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

        DD::Image::ChannelSet flatten_channels = out_channels;
        flatten_channels += DD::Image::Chan_Alpha; // always need alpha!
        flatten_channels -= DD::Image::Mask_Deep;
        flatten_channels -= DD::Image::Mask_Z;
        flatten_channels -= m_spmask_channels;

        Dcx::ChannelSet dcx_flatten_channels(Dcx::dcxChannelSet(flatten_channels));
        Dcx::ChannelSet dcx_out_channels(Dcx::dcxChannelSet(out_channels));

        std::vector<DD::Image::Channel> spmask8_chan_list(2); // to pass to dcxCopyDDImageDeepPixel()
        spmask8_chan_list[0] = k_spmask_channel[0];
        spmask8_chan_list[1] = k_spmask_channel[1];

        Dcx::Pixelf dcx_flattened(dcx_out_channels);

        Dcx::DeepPixel dcx_deep_pixel(dcx_flatten_channels);

        // Don't use Row::erase(),we want to fill the row memory:
        foreach(z, out_channels)
            memset(out_row.writable(z)+x, 0, (r-x)*sizeof(float));
        foreach(z, out_z_channels) {
            float* OUT = out_row.writable(z)+x;
            for (int X=x; X < r; ++X)
                *OUT++ = INFINITYf;
        }

        // If there's no subpixel masks we only need to flatten once per-pixel:
        if (m_spmask_size == 0) {
            //------------------------------------
            // No subpixel masks
            //------------------------------------
            for (int X=x; X < r; ++X) {
#ifdef DCX_DEBUG_FLATTENER
                const bool debug = (X == int(k_debug[0]) && y == int(k_debug[1]));
#endif
                Dcx::dcxCopyDDImageDeepPixel(in_deep_plane.getPixel(y, X),
                                             (Dcx::SpMaskMode)k_spmask_mode,
                                             spmask8_chan_list,
                                             k_flags_channel,
                                             dcx_deep_pixel);
#ifdef DCX_DEBUG_FLATTENER
                if (debug) dcx_deep_pixel.printInfo(std::cout);
#endif

                // Skip if nothing to flatten:
                if (dcx_deep_pixel.size() == 0)
                    continue;

                dcx_deep_pixel.flatten(dcx_flatten_channels,
                                       dcx_flattened,
                                       (Dcx::InterpolationMode)k_interpolation_mode);

                // Copy flattened pixel to output row:
                foreach(z, out_channels)
                    out_row.writable(z)[X] = dcx_flattened[Dcx::dcxChannel(z)];
            }

        } else {
            //------------------------------------
            // Subpixel masks
            //------------------------------------
            Dcx::Pixelf dcx_accum(dcx_flatten_channels);

            for (int X=x; X < r; ++X) {
#ifdef DCX_DEBUG_FLATTENER
                const bool debug = (X == int(k_debug[0]) && y == int(k_debug[1]));
#endif
                dcx_accum.erase();
                //dcx_accum[Dcx::Chan_Z      ] =  INFINITYf;
                dcx_accum[Dcx::Chan_ZFront ] =  INFINITYf;
                dcx_accum[Dcx::Chan_ZBack  ] = -INFINITYf;
                //dcx_accum[Dcx::Chan_CutoutA] =  0.0f;
                dcx_accum[Dcx::Chan_CutoutZ] =  INFINITYf;

                Dcx::dcxCopyDDImageDeepPixel(in_deep_plane.getPixel(y, X),
                                             (Dcx::SpMaskMode)k_spmask_mode,
                                             spmask8_chan_list,
                                             k_flags_channel,
                                             dcx_deep_pixel);
#ifdef DCX_DEBUG_FLATTENER
                if (debug) dcx_deep_pixel.printInfo(std::cout);
#endif

                // Skip if nothing to flatten:
                if (dcx_deep_pixel.size() == 0)
                    continue;

                // If all segments are full coverage then only flatten only once:
                const int subpixels = (dcx_deep_pixel.allFullCoverage())?1:m_spmask_size;

                Dcx::SpMask8 sp_mask(1ull);
                int count = 0;
                for (int sp_y=0; sp_y < subpixels; ++sp_y) {
                    for (int sp_x=0; sp_x < subpixels; ++sp_x) {

                        dcx_deep_pixel.flattenSubpixels(dcx_flatten_channels, dcx_flattened, sp_mask/*spmask*/,
                                                        (Dcx::InterpolationMode)k_interpolation_mode/*interp_mode*/);
                        // Add flattended subpixel to accumulation pixel:
                        dcx_accum += dcx_flattened;

                        dcx_accum[Dcx::Chan_ZFront] = std::min(dcx_accum[Dcx::Chan_ZFront], dcx_flattened[Dcx::Chan_ZFront]);
#if 0
                        // TODO: This causes gcc to crash...specifically writing to Chan_ZBack, or index 12...
                        // 11 is fine, 13 is fine, only 12 crashes...what the heck...?
                        //dcx_accum[12] = dcx_accum[Dcx::Chan_ZFront];
                        //dcx_accum[Dcx::Chan_ZBack] = dcx_accum[Dcx::Chan_ZFront];
                        dcx_accum[Dcx::Chan_ZBack ] = std::min(dcx_accum[Dcx::Chan_ZBack ], dcx_flattened[Dcx::Chan_ZBack ]);
#endif
                        dcx_accum[Dcx::Chan_CutoutZ] = std::min(dcx_flattened[Dcx::Chan_CutoutZ], dcx_accum[Dcx::Chan_CutoutZ]);
                        // Make sure ZB is always legal:
                        if (dcx_accum[Dcx::Chan_ZBack] < dcx_accum[Dcx::Chan_ZFront])
                            dcx_accum[Dcx::Chan_ZBack] = dcx_accum[Dcx::Chan_ZFront];

                        ++sp_mask; // same as <<=
                        ++count;
                    }
                }
#ifdef DEBUG
                assert(count > 0); // Shouldn't happen...
#endif

                // If final cutout Z is in front of non-cutout Z, output INF:
                if (k_cutout_z && dcx_accum[Dcx::Chan_CutoutZ] < dcx_accum[Dcx::Chan_ZFront])
                    dcx_accum[Dcx::Chan_ZFront] = dcx_accum[Dcx::Chan_ZBack] = INFINITYf;

                // Write flattened result to Row:
                if (count > 1) {
                    const float iw = 1.0f / float(count);
                    foreach(z, flatten_channels)
                        out_row.writable(z)[X] = dcx_accum[Dcx::dcxChannel(z)]*iw;
                } else {
                    foreach(z, flatten_channels)
                        out_row.writable(z)[X] = dcx_accum[Dcx::dcxChannel(z)];
                }
                foreach(z, out_z_channels)
                    out_row.writable(z)[X] = dcx_accum[Dcx::dcxChannel(z)];

            } // x-r pixel loop

        } // have subpixel masks?

        // Apply 1/Z now if desired, but only to Chan_Z:
        if (k_one_over_z && out_channels.contains(DD::Image::Chan_Z)) {
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
