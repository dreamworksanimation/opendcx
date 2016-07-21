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

using namespace DD::Image;

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

/*!
*/
class DeepTransform : public DeepFilterOp, public Dcx::DeepTransform {
    Channel                 k_spmask_channel[2];    //!< 
    DD::Image::Channel      k_flags_channel;        //!< Per-sample flags channel
    //
    Matrix4                 k_transform;
    //double                  k_ztranslate;
    //double                  k_zscale;
    //bool                    k_invert;
    //
    Filter                  k_filter;               //!< Filter to use
    bool                    k_black_outside;        //!< 
#ifdef DCX_DEBUG_TRANSFORM
    float                   k_debug[2];
#endif

    ChannelSet              m_input_spmask_channels;

public:
    static const Description description;
    /*virtual*/ const char* Class() const { return description.name; }
    /*virtual*/ const char* node_help() const { return __DATE__ " " __TIME__ "\n"
        "This the deep version of Transform with support for subpixel mask resampling";
    }
    /*virtual*/ const char* displayName() const { return "DeepTransform"; }

    DeepTransform(Node* node) :
        DeepFilterOp(node),
        Dcx::DeepTransform(Dcx::DeepTransform::FILTER_BOX), 
        k_filter(DD::Image::Filter::Cubic)
    {
        k_spmask_channel[0]  = DD::Image::getChannel(Dcx::spMask8ChannelName1);
        k_spmask_channel[1]  = DD::Image::getChannel(Dcx::spMask8ChannelName2);
        k_flags_channel      = DD::Image::getChannel(Dcx::flagsChannelName);
        //
        k_transform.makeIdentity();
        //k_ztranslate    = 0.0;
        //k_zscale        = 1.0;
        //k_invert        = false;
        //
        m_ss_factor     = 4;
        k_black_outside = true;
#ifdef DCX_DEBUG_TRANSFORM
        k_debug[0] = k_debug[1] = -1.0f;
#endif
    }

    /*virtual*/ Op* op() { return this; }

    /*! */
    /*virtual*/
    void knobs(Knob_Callback f) {
        Transform2d_knob(f, &k_transform, "transform", "transform");
        //
        //Double_knob(f, &k_ztranslate, "ztranslate", "");
        //    ClearFlags(f, Knob::SLIDER);
        //    SetFlags(f, Knob::LOG_SLIDER);
        //Double_knob(f, &k_zscale, IRange(0.0,5.0), "zscale", "zscale");
        //    ClearFlags(f, Knob::SLIDER || Knob::STARTLINE);
        //    SetFlags(f, Knob::LOG_SLIDER);
        //    Tooltip(f, "Deep scale.  This is a divisor, so number > 1 will reduce the depth "
        //                "and numbers < 1 will increase the depth.");
        Newline(f);
        //
        Int_knob(f, &m_ss_factor, "supersampling_factor", "supersampling factor");
        k_filter.knobs(f, "filter", "filter");
        Bool_knob(f, &k_black_outside, "black_outside", "black outside");
            Tooltip(f, "Crop the picture with an anti-aliased black edge at the bounding box.");
        //
        Divider(f);
        Input_Channel_knob(f, k_spmask_channel, 2/*nChans*/, 0/*input*/, "spmask_channels", "spmask channels");
            Tooltip(f, "Channels which contain the per-sample spmask data. Two channels are required for an 8x8 mask.");
        Input_Channel_knob(f, &k_flags_channel, 1/*nChans*/, 0/*input*/, "flags_channel", "flags");
            ClearFlags(f, Knob::STARTLINE);
            SetFlags(f, Knob::NO_CHECKMARKS);
            Tooltip(f, "Channel which contains the per-sample flag data.");

        Obsolete_knob(f, "mask_channel", 0);
#ifdef DCX_DEBUG_TRANSFORM
        Divider(f);
        XY_knob(f, k_debug, "debug", "debug");
#endif
    }

    /*! */
    /*virtual*/
    void _validate(bool for_real) {
        DeepFilterOp::_validate(for_real);

        // Get subpixel mask channels:
        m_input_spmask_channels = Mask_None;
        if (k_filter.type() != Filter::Impulse) {
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
        Dcx::DeepTransform::setMatrix(Imath::M44f(m44f));

        // Forward-transform the output bbox:
        // TODO: do we need any pad here...?
        const Box& input_bbox = _deepInfo.box();
        Imath::Box2i ob = Dcx::DeepTransform::transform(Imath::Box2i(Imath::V2i(input_bbox.x(),   input_bbox.y()  ),
                                                                     Imath::V2i(input_bbox.r()-1, input_bbox.t()-1)));
        const int pad = (k_black_outside)?2:1;
        Box output_bbox(ob.min.x-pad, ob.min.y-pad, ob.max.x+pad+1, ob.max.y+pad+1);

        ChannelSet output_channels(_deepInfo.channels());

        if (k_filter.type() != Filter::Impulse) {
            Dcx::DeepTransform::m_filter_mode = Dcx::DeepTransform::FILTER_BOX;
            // Output the spmask channels even if they don't exist on input:
            output_channels += k_spmask_channel[0];
            output_channels += k_spmask_channel[1];
            output_channels += k_flags_channel;
        } else {
            Dcx::DeepTransform::m_filter_mode = Dcx::DeepTransform::FILTER_NEAREST;
        }

        _deepInfo = DeepInfo(_deepInfo.formats(), output_bbox, output_channels);
    }

    //--------------------------------------------------------------------------------------------

    /*!
    */
    /*virtual*/
    void getDeepRequests(Box bbox, const ChannelSet& channels, int count, std::vector<RequestData>& requests) {
        if (!input0())
            return;

        // Expand the request to the input format and spmask channels:
        Imath::Box2i ib = Dcx::DeepTransform::backTransform(Imath::Box2i(Imath::V2i(bbox.x(),   bbox.y()  ),
                                                                         Imath::V2i(bbox.r()-1, bbox.t()-1)));
        Box input_bbox(ib.min.x, ib.min.y, ib.max.x+1, ib.max.y+1);
        input_bbox.pad(1); // just in case...
        input_bbox.intersect(input0()->deepInfo().box());

        ChannelSet input_channels(channels);
        input_channels += m_input_spmask_channels;
#if 1
        requests.push_back(RequestData(input0(), input_bbox, input_channels, count));
#else
        DeepFilterOp::getDeepRequests(input_bbox, input_channels, count, requests);
#endif
    }

    //--------------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------------

    /*!
    */
    /*virtual*/
    bool doDeepEngine(Box output_bbox, const ChannelSet& output_channels, DeepOutputPlane& deep_out_plane) {
        if (!input0())
            return true;

        deep_out_plane = DeepOutputPlane(output_channels, output_bbox);

        Imath::Box2i ob = Dcx::DeepTransform::backTransform(Imath::Box2i(Imath::V2i(output_bbox.x(),   output_bbox.y()  ),
                                                                         Imath::V2i(output_bbox.r()-1, output_bbox.t()-1)));
        Box input_bbox(ob.min.x, ob.min.y, ob.max.x+1, ob.max.y+1);

        // If input box is inversed output black:
        //TODO: don't think this can ever happen, check Dcx::DeepTransform
        if (input_bbox.x() >= input_bbox.r() || input_bbox.y() >= input_bbox.t()) {
            for (Box::iterator it = output_bbox.begin(); it != output_bbox.end(); ++it)
                deep_out_plane.addHole();
            return true;
        }

#ifdef DCX_DEBUG_TRANSFORM
        const bool debugY = false;//(output_bbox.begin().y == 78*3);
        if (debugY) {
            std::cout << "DeepTransform::doDeepEngine() output_channels=" << output_channels;
            std::cout << ", output_bbox[" << output_bbox.x() << " " << output_bbox.y() << " " << output_bbox.r() << " " << output_bbox.t() << "]";
            std::cout << ", input_bbox[" << input_bbox.x() << " " << input_bbox.y() << " " << input_bbox.r() << " " << input_bbox.t() << "]";
            std::cout << std::endl;
        }
#endif

        DeepPlane deep_in_plane;
        ChannelSet input_channels = output_channels;
        input_channels += m_input_spmask_channels;
        if (!input0()->deepEngine(input_bbox, input_channels, deep_in_plane))
            return false;

        // Wrap the in/out DeepPlanes in Dcx::DeepTiles.
        // This is cheap - it doesn't copy any data, only res & channel info.
        //
        std::vector<DD::Image::Channel> spmask8_chan_list(2); // to pass to dcxCopyDDImageDeepPixel()
        spmask8_chan_list[0] = k_spmask_channel[0];
        spmask8_chan_list[1] = k_spmask_channel[1];
        Dcx::DDImageDeepInputPlane   dcx_in_tile(*_deepInfo.format(),  &deep_in_plane, Dcx::SPMASK_AUTO, spmask8_chan_list, k_flags_channel);
        Dcx::DDImageDeepOutputPlane dcx_out_tile(*_deepInfo.format(), &deep_out_plane, Dcx::SPMASK_AUTO, spmask8_chan_list, k_flags_channel);

        Dcx::ChannelSet xform_channels(dcx_in_tile.channels());
        xform_channels &= Dcx::Mask_RGBA; // TODO: fix this - do we really need to filter spmask & flags channels?  DeepTransform/DeepTile should do this.

        Dcx::DeepPixel out_pixel(xform_channels);
        out_pixel.reserve(10);

        for (Box::iterator it = output_bbox.begin(); it != output_bbox.end(); ++it) {
            if (Op::aborted())
                return false; // bail fast on user-interrupt

#ifdef DCX_DEBUG_TRANSFORM
            const bool debug = (it.x == int(k_debug[0]) && it.y == int(k_debug[1]));
#endif

            Dcx::DeepTransform::sample(it.x, it.y, dcx_in_tile, out_pixel);
#ifdef DCX_DEBUG_TRANSFORM
            if (debug)
                out_pixel.printInfo(std::cout, "out_pixel=", 1/*padding*/);
#endif

            dcx_out_tile.setDeepPixel(-1/*x ignored*/, -1/*y ignored*/, out_pixel);
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
