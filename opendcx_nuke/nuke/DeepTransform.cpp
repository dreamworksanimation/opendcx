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
/// @file DeepTransform.cpp

#include <DDImage/DeepFilterOp.h>
#include "DDImage/Filter.h"
#include "DDImage/Knob.h"
#include "DDImage/Knobs.h"
#include "DDImage/ViewerContext.h"
#include "DDImage/MetaData.h"

#include <OpenDCX/DcxDeepTransform.h>

#include "DDImageAdapter.h"

// Uncomment this to get debug info:
//#define DCX_DEBUG_TRANSFORM 1
#ifdef DCX_DEBUG_TRANSFORM
static bool show_debug = true;
#else
static bool show_debug = false;
#endif

using namespace DD::Image;

//
//  DeepTransform
//
//      An example Nuke plugin that applies a 2D affine transformation to all deep pixels
//      in the image. If the source deep samples have subpixel masks then they are
//      spatially resampled using the supersampling rate.
//
//      If the filter is not impulse and a deep sample has a partial subpixel contribution
//      resulting from the transform then multiple copies of the sample are written to the
//      output deep pixel (which may get combined with another sample) - one with the opaque
//      contribution and one or more with the partially-weighted contributions. The subpixel
//      masks of these partial samples are mutually exclusive with the opaque sample so they
//      never overlap, and are flagged as having partial-coverage. The flattener algorithm
//      accumulates these samples additively before combining with the opaque sample.
//
//      This node can replace the stock DeepTransform node with the caveat that the stock
//      node uses an XYZ knob for translate which does not map directly to the
//      Transform2d_knob's XY translate, causing saved Z-translate values to be *LOST* from
//      preexisting scripts!
//
//  TODO:
//    * Support filter kernels selected by user.  At the moment the Dcx::DeepTransform
//      class only supports box or no(impulse) filtering.
//
//    * Add controls to limit the number of additional partial-spcoverage samples created
//      by subpixel transforms. If the transform consists of only translate or integer
//      scale then a few additional samples are created, however rotation can end up
//      adding many more, up to ss_rate*ss_rate per sample...
//      Dcx::DeepTransform should have a max limit on the number of partial-coverage
//      bins as most of the time there's only slight differences in the coverage
//      weights and a smaller set is perfectly fine.  Also consider making the
//      bin distribution perceptually biased.
//
//    * Remove debug switches eventually
//

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// Keep this matching Dcx::DeepTransform::SuperSamplingRate enums:
static const char* ss_rate_modes[] = { "1", "2", "4", "8", "16", /*"32",*/ 0 };

// TODO: disable this and implement true filter kernels:
#define TEMP_FILTER_CONTROLS 1
#ifdef TEMP_FILTER_CONTROLS
// Filter mode standins:
static const char* temp_filter_modes[] = { "Impulse", "Box", 0 };
#endif

/*!
*/
class DeepTransform : public DeepFilterOp, public Dcx::DeepTransform {
    DD::Image::Channel      k_spmask_channel[2];    //!< Per-sample subpixel mask channels
    DD::Image::Channel      k_flags_channel;        //!< Per-sample flags channel
    //
    Matrix4                 k_transform;            //!< 4x4 matrix (no Z is used though) filled in by Transform2d knob
    double                  k_ztranslate;           //!<
    double                  k_zscale;               //!< 
    bool                    k_invert;
    //
#ifdef TEMP_FILTER_CONTROLS
    int                     k_filter;
#else
    Filter                  k_filter;               //!< Filter to use (TODO: implement filter kernel sampling)
#endif
    int                     k_ss_mode;
    bool                    k_black_outside;        //!< Pad the output bbox with a single pixel of black
    //
    float                   k_debug[2];

    DD::Image::ChannelSet   m_input_spmask_channels;

public:
    static const Description description;
    /*virtual*/ const char* Class() const { return description.name; }
    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "This the deep version of Transform with support for subpixel mask resampling";
    }
    /*virtual*/ const char* displayName() const { return "DeepTransform"; }

    DeepTransform(Node* node) :
        DeepFilterOp(node),
        Dcx::DeepTransform()
#ifdef TEMP_FILTER_CONTROLS
#else
        , k_filter(DD::Image::Filter::Cubic)
#endif
    {
        // Get OpenDCX standard channels assigned in the correct order:
        Dcx::dcxGetSpmaskChannels(k_spmask_channel[0], k_spmask_channel[1], k_flags_channel);
        //
        k_transform.makeIdentity();
        k_ztranslate        = 0.0;     // Apply ztranslate, then zscale
        k_zscale            = 1.0;
        k_invert            = false;
        //
#ifdef TEMP_FILTER_CONTROLS
        k_filter            = 1; // Box
#endif
        k_ss_mode           = (int)SS_RATE_4;
        k_black_outside     = true;
        //
        k_debug[0] = k_debug[1] = -1.0f; // no debug
    }

    /*virtual*/ Op* op() { return this; }

    /*! */
    /*virtual*/
    void knobs(Knob_Callback f) {
        Double_knob(f, &k_ztranslate, "ztranslate", "depth translate");
            ClearFlags(f, Knob::SLIDER);
            Tooltip(f, "Depth translate. Negative values move samples closer to camera.");
        Double_knob(f, &k_zscale, IRange(0.0,5.0), "zscale", "scale");
            ClearFlags(f, Knob::SLIDER | Knob::STARTLINE);
            Tooltip(f, "Depth scale.  This is a divisor, so number > 1 will reduce the depth "
                        "and numbers < 1 will increase the depth.");
        Divider(f);
        Transform2d_knob(f, &k_transform, "transform", "transform");
        Newline(f);
        Bool_knob(f, &k_invert, "invert_xform", "invert");
            Tooltip(f, "Invert the 2D and depth transform.");
        //
#ifdef TEMP_FILTER_CONTROLS
        Enumeration_knob(f, &k_filter, temp_filter_modes, "filter", "filter");
            Tooltip(f, "(temp standin menu for real filter options)</b>"
                        "<ul>"
                        "<li><i>Impulse</i> - no filtering, each output pixel equals some input pixel.</li>"
                        "<li><i>Box</i> - average filtering of all input pixels contributing to output pixel.</li>"
                        "</ul>"
                    );
#else
        k_filter.knobs(f, "filter", "filter");
#endif
        Bool_knob(f, &k_black_outside, "black_outside", "black outside");
            Tooltip(f, "Crop the picture with an anti-aliased black edge at the bounding box.");
        //
        Newline(f);
        Enumeration_knob(f, &k_ss_mode, ss_rate_modes, "supersampling", "supersampling");
            Tooltip(f, "Sampling rate for subpixel mask reconstruction.\n"
                        "Higher rates can dramatically increase the number of output deep samples, especially "
                        "from rotation or subpixel translates and scales.");
        Divider(f);
        Input_Channel_knob(f, k_spmask_channel, 2/*nChans*/, 0/*input*/, "spmask_channels", "spmask channels");
            Tooltip(f, "Channels which contain the per-sample spmask data. Two channels are required for an 8x8 mask.");
        Input_Channel_knob(f, &k_flags_channel, 1/*nChans*/, 0/*input*/, "flags_channel", "flags");
            ClearFlags(f, Knob::STARTLINE);
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channel which contains the per-sample flag data.");

        Obsolete_knob(f, "mask_channel", 0);
        //
        if (show_debug)
            Divider(f);
        XY_knob(f, k_debug, "debug", (show_debug)?"debug":INVISIBLE);
    }

    /*! */
    /*virtual*/
    void _validate(bool for_real) {
        DeepFilterOp::_validate(for_real);

        // Get subpixel mask channels:
        m_input_spmask_channels = Mask_None;
#ifdef TEMP_FILTER_CONTROLS
        if (k_filter != 0)
#else
        if (k_filter.type() != Filter::Impulse)
#endif
        {
            for (unsigned j=0; j < 2; ++j) {
                if (k_spmask_channel[j] != Chan_Black && _deepInfo.channels().contains(k_spmask_channel[j]))
                    m_input_spmask_channels += k_spmask_channel[j];
            }
        }
        // We always want the flags channel:
        m_input_spmask_channels += k_flags_channel;

        // Setting the matrix this way makes sure inverse matrix gets updated:
        float m44f[4][4];
        memcpy(m44f[0], k_transform.array(), 16*sizeof(float));
        if (!k_invert)
            Dcx::DeepTransform::setMatrix(Imath::M44f(m44f));
        else
            Dcx::DeepTransform::setMatrix(Imath::M44f(m44f).inverse());

        // Forward-transform the output bbox:
        const Box& input_bbox = _deepInfo.box();
        Imath::Box2i ob = Dcx::DeepTransform::transform(Imath::Box2i(Imath::V2i(input_bbox.x(),   input_bbox.y()  ),
                                                                     Imath::V2i(input_bbox.r()-1, input_bbox.t()-1)));
#ifdef DCX_DEBUG_TRANSFORM
        std::cout << "  in_bbox[" << input_bbox.x() << " " << input_bbox.y() << " " << input_bbox.r()-1 << " " << input_bbox.t()-1 << "]" << std::endl;
        std::cout << "  invert=" << k_invert << std::endl;
        std::cout << "  transform:" << std::endl << std::fixed << Dcx::DeepTransform::matrix();
        std::cout << "  out_bbox[" << ob.min.x << " " << ob.min.y << " " << ob.max.x << " " << ob.max.y << "]" << std::endl;
#endif

        // Pad outut by one pixel, or 2 if we want black outside:
        const int pad = (k_black_outside)?2:1;
        Box output_bbox(ob.min.x-pad, ob.min.y-pad, ob.max.x+pad+1, ob.max.y+pad+1);

        DD::Image::ChannelSet output_channels(_deepInfo.channels());

        // TODO: support other DDImage::Filter types:
#ifdef TEMP_FILTER_CONTROLS
        if (k_filter != 0 && k_ss_mode > 0)
#else
        if (k_filter.type() != Filter::Impulse && k_ss_mode > 0)
#endif
        {
            Dcx::DeepTransform::m_filter_mode = Dcx::DeepTransform::FILTER_BOX;
            setSuperSamplingMode((SuperSamplingMode)k_ss_mode);
            // Output the spmask channels even if they don't exist on input:
            output_channels += k_spmask_channel[0];
            output_channels += k_spmask_channel[1];
            output_channels += k_flags_channel;
        } else {
            Dcx::DeepTransform::m_filter_mode = Dcx::DeepTransform::FILTER_NEAREST;
            setSuperSamplingMode(SS_RATE_1);
        }

        _deepInfo = DeepInfo(_deepInfo.formats(), output_bbox, output_channels);
    }

    //--------------------------------------------------------------------------------------------

    /*!
    */
    /*virtual*/
    void getDeepRequests(Box bbox, const DD::Image::ChannelSet& channels, int count, std::vector<RequestData>& requests) {
        if (!input0())
            return;

        // Expand the request to the input format and spmask channels:
        Imath::Box2i ib = Dcx::DeepTransform::backTransform(Imath::Box2i(Imath::V2i(bbox.x(),   bbox.y()  ),
                                                                         Imath::V2i(bbox.r()-1, bbox.t()-1)));
        Box input_bbox(ib.min.x, ib.min.y, ib.max.x+1, ib.max.y+1);
        input_bbox.pad(1); // just in case...
        input_bbox.intersect(input0()->deepInfo().box());

        DD::Image::ChannelSet input_channels(channels);
        input_channels += m_input_spmask_channels;
        requests.push_back(RequestData(input0(), input_bbox, input_channels, count));
    }

    //--------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------

    /*!
    */
    /*virtual*/
    bool doDeepEngine(Box output_bbox, const DD::Image::ChannelSet& output_channels, DeepOutputPlane& deep_out_plane) {
        if (!input0())
            return true;

        deep_out_plane = DeepOutputPlane(output_channels, output_bbox);

        Imath::Box2i ob = Dcx::DeepTransform::backTransform(Imath::Box2i(Imath::V2i(output_bbox.x(),   output_bbox.y()  ),
                                                                         Imath::V2i(output_bbox.r()-1, output_bbox.t()-1)));
        Box input_bbox(ob.min.x, ob.min.y, ob.max.x+1, ob.max.y+1);

#ifdef DCX_DEBUG_TRANSFORM
        const bool debugY = (output_bbox.begin().y == int(k_debug[1]));
        if (debugY) {
            std::cout << "DeepTransform::doDeepEngine() output_channels=" << output_channels;
            std::cout << ", output_bbox[" << output_bbox.x() << " " << output_bbox.y() << " " << output_bbox.r() << " " << output_bbox.t() << "]";
            std::cout << ", input_bbox[" << input_bbox.x() << " " << input_bbox.y() << " " << input_bbox.r() << " " << input_bbox.t() << "]";
            std::cout << std::endl;
        }
#endif

        DD::Image::DeepPlane deep_in_plane;
        DD::Image::ChannelSet input_channels = output_channels;
        input_channels += m_input_spmask_channels;
        if (!input0()->deepEngine(input_bbox, input_channels, deep_in_plane))
            return false;

        //
        // Wrap the in/out DeepPlanes in Dcx::DeepTiles.
        // This is cheap - it doesn't copy any pixel data, only res & channel info.
        //
        std::vector<DD::Image::Channel> spmask8_chan_list(2); // to pass to dcxCopyDDImageDeepPixel()
        spmask8_chan_list[0] = k_spmask_channel[0];
        spmask8_chan_list[1] = k_spmask_channel[1];
        Dcx::DDImageDeepInputPlane   dcx_in_tile(*_deepInfo.format(),  &deep_in_plane, k_spmask_channel[0], k_spmask_channel[1], k_flags_channel);
        Dcx::DDImageDeepOutputPlane dcx_out_tile(*_deepInfo.format(), &deep_out_plane, k_spmask_channel[0], k_spmask_channel[1], k_flags_channel);

        Dcx::ChannelSet xform_channels(dcx_in_tile.channels());
        xform_channels &= Dcx::Mask_RGBA; // TODO: fix this - do we really need to filter spmask & flags channels?  DeepTransform/DeepTile should do this.

        Dcx::DeepPixel out_pixel(xform_channels);
        out_pixel.reserve(10);

        //
        // Iterate through the output plane sequentially, transforming
        // and writing the sampled deep pixel.
        //
        for (Box::iterator it = output_bbox.begin(); it != output_bbox.end(); ++it) {
            if (Op::aborted())
                return false; // bail fast on user-interrupt

#ifdef DCX_DEBUG_TRANSFORM
            const bool debug = (it.x == int(k_debug[0]) && it.y == int(k_debug[1]));
#endif

            Dcx::DeepTransform::sample(it.x, it.y, dcx_in_tile, xform_channels, out_pixel);
#if 0//def DCX_DEBUG_TRANSFORM
            if (debug) out_pixel.printInfo(std::cout, "out_pixel=", 1/*padding*/, true/*show_mask*/);
#endif

            // Apply Z transforms:
            if (!k_invert)
                out_pixel.transformDepths(k_ztranslate, 1.0/k_zscale, false);
            else
                out_pixel.transformDepths(-k_ztranslate, k_zscale, true);

            dcx_out_tile.setDeepPixelSequential(out_pixel);
        }

        return true;
    }

};


static Op* build(Node* node) { return new DeepTransform(node); }
const Op::Description DeepTransform::description("DeepTransform", "Deep/DeepTransform", build);

// end of DeepTransform.cpp

// TM and (c) 2015-2016 DreamWorks Animation LLC.  All Rights Reserved.
// Reproduction in whole or in part without prior written permission of a
// duly authorized representative is prohibited.
