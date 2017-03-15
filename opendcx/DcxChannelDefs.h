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
/// @file DcxChannelDefs.h

#ifndef INCLUDED_DCX_CHANNELDEFS_H
#define INCLUDED_DCX_CHANNELDEFS_H

#include "DcxChannelSet.h"


//----------------
//!rst:cpp:begin::
//ChannelDefs
//===========
//----------------


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//=============================================================================
//
// Default channel names for deep metadata IO
//
// IMPORTANT NOTE: these names have changed from the original DigiPro2015
// paper due to Nuke 10 not supporting numeric channel names! Specifically Nuke
// no longer supports layer or channel names which start with a number.
//
// Additionally the channel type for the flags channel is now 32-bit float.
//
// To avoid channel reordering conflicts in applications it's best to also
// change the layer name. Here's the old to new name mappings:
//   old (2.2.1)     new (2.2.2)
//  spmask.1   ->   deepabuf.sp1
//  spmask.2   ->   deepabuf.sp2
//  spmask.3   ->   deepmeta.flags
//
//=============================================================================

const char* const spMask8LayerName     = "deepabuf";
const char* const spMask8Chan1Name     = "sp1";
const char* const spMask8Chan2Name     = "sp2";
const char* const spMask8Channel1Name  = "deepabuf.sp1";
const char* const spMask8Channel2Name  = "deepabuf.sp2";
static const OPENEXR_IMF_NAMESPACE::PixelType spMask8ChannelType = OPENEXR_IMF_NAMESPACE::FLOAT;
//
const char* const flagsLayerName       = "deepmeta";
const char* const flagsChanName        = "flags";
const char* const flagsChannelName     = "deepmeta.flags";
static const OPENEXR_IMF_NAMESPACE::PixelType flagsChannelType = OPENEXR_IMF_NAMESPACE::FLOAT;


//=============================================================================
//
// Predefined indices for recommended standard channel definitions
// See OpenEXR TechnicalIntroduction.pdf, pages 19-20
//
//=============================================================================

static const ChannelIdx  Chan_R             =  1; // Red
static const ChannelIdx  Chan_G             =  2; // Green
static const ChannelIdx  Chan_B             =  3; // Blue
//
static const ChannelIdx  Chan_A             =  4; // Alpha
static const ChannelIdx  Chan_AR            =  5; // Red-opacity
static const ChannelIdx  Chan_AG            =  6; // Green-opacity
static const ChannelIdx  Chan_AB            =  7; // Blue-opacity
//
static const ChannelIdx  Chan_Y             =  8; // Luminance
static const ChannelIdx  Chan_RY            =  9; // Chroma red-minus-Y
static const ChannelIdx  Chan_BY            = 10; // Chroma blue-minus-Y
//
static const ChannelIdx  Chan_Z             = 11; // Z-depth distance from camera
static const ChannelIdx  Chan_ZFront        = 11; // Same as Chan_Z for the moment
static const ChannelIdx  Chan_ZBack         = 12; // Distance of the back of a (deep) sample from the viewer
//
static const ChannelIdx  Chan_SpBits1       = 13; // Subpixel bitmask channel 1 - 1+2 req for 8x8 mask
static const ChannelIdx  Chan_SpBits2       = 14; // Subpixel bitmask channel 2 - 1+2 req for 8x8 mask
static const ChannelIdx  Chan_DeepFlags     = 15; // Per-sample DeepFlags (and partial subpixel-coverage weight)
//
static const ChannelIdx  Chan_ACutout       = 16; // Alpha with cutouts applied (holes) - used during front-to-back flattening
static const ChannelIdx  Chan_ARCutout      = 17; // Red-opacity with cutouts applied
static const ChannelIdx  Chan_AGCutout      = 18; // Green-opacity with cutouts applied
static const ChannelIdx  Chan_ABCutout      = 19; // Blue-opacity with cutouts applied
static const ChannelIdx  Chan_ZCutout       = 20; // Depth with cutouts applied
//
static const ChannelIdx  Chan_Visibility    = 21; // Accumlated visibility - RESERVED, used during front-to-back flattening
static const ChannelIdx  Chan_SpCoverage    = 22; // Accumulated subpixel-coverage - RESERVED, used during front-to-back flattening
//
static const ChannelIdx  Chan_UvS           = 23; // Texture coordinate S
static const ChannelIdx  Chan_UvT           = 24; // Texture coordinate T
static const ChannelIdx  Chan_UvP           = 25; // Texture coordinate P
static const ChannelIdx  Chan_UvQ           = 26; // Texture coordinate Q
//
static const ChannelIdx  Chan_ID0           = 27; // Numerical identifier for an object/surface
static const ChannelIdx  Chan_ID1           = 28; // Numerical identifier for an object/surface
static const ChannelIdx  Chan_ID2           = 29; // Numerical identifier for an object/surface
static const ChannelIdx  Chan_ID3           = 30; // Numerical identifier for an object/surface
//
// Arbitrary channel indices start at this one:
static const ChannelIdx  Chan_ArbitraryStart = Chan_ID3+1;


//=============================================================================
//
// Predefined ChannelSets(masks) for recommended standard channel definitions
// See OpenEXR TechnicalIntroduction.pdf, pages 19-20
//
//=============================================================================

static const ChannelSet  Mask_None;
//
static const ChannelSet  Mask_R (Chan_R);
static const ChannelSet  Mask_G (Chan_G);
static const ChannelSet  Mask_B (Chan_B);
static const ChannelSet  Mask_A (Chan_A);
static const ChannelSet  Mask_RGB (Chan_R, Chan_G, Chan_B);
static const ChannelSet  Mask_RGBA (Chan_R, Chan_G, Chan_B, Chan_A);
//
static const ChannelSet  Mask_AR (Chan_AR);
static const ChannelSet  Mask_AG (Chan_AG);
static const ChannelSet  Mask_AB (Chan_AB);
static const ChannelSet  Mask_RGBAlphas (Chan_AR, Chan_AG, Chan_AB);
static const ChannelSet  Mask_Alphas (Chan_A, Chan_AR, Chan_AG, Chan_AB);
static const ChannelSet  Mask_Opacities (Chan_A, Chan_SpCoverage);
static const ChannelSet  Mask_RGBOpacities (Chan_AR, Chan_AG, Chan_AB, Chan_SpCoverage);
//
static const ChannelSet  Mask_Y (Chan_Y);
static const ChannelSet  Mask_RY (Chan_RY);
static const ChannelSet  Mask_BY (Chan_BY);
static const ChannelSet  Mask_Yuv (Chan_Y, Chan_RY, Chan_BY);
//
static const ChannelSet  Mask_ZFront (Chan_ZFront);
static const ChannelSet  Mask_ZBack (Chan_ZBack);
static const ChannelSet  Mask_Depths (Chan_ZFront, Chan_ZBack);
//
static const ChannelSet  Mask_SpBits1 (Chan_SpBits1);
static const ChannelSet  Mask_SpBits2 (Chan_SpBits2);
static const ChannelSet  Mask_SpMask8 (Chan_SpBits1, Chan_SpBits2);
static const ChannelSet  Mask_DeepFlags (Chan_DeepFlags);
static const ChannelSet  Mask_DeepMetadata (Chan_DeepFlags, Chan_SpBits1, Chan_SpBits2);
//
static const ChannelSet  Mask_Visibility (Chan_Visibility);
static const ChannelSet  Mask_SpCoverage (Chan_SpCoverage);
static const ChannelSet  Mask_Accumulators (Chan_Visibility);
//
static const ChannelSet  Mask_ACutout (Chan_ACutout);
static const ChannelSet  Mask_ARCutout (Chan_ARCutout);
static const ChannelSet  Mask_AGCutout (Chan_AGCutout);
static const ChannelSet  Mask_ABCutout (Chan_ABCutout);
static const ChannelSet  Mask_ACutoutlphas (Chan_ACutout, Chan_ARCutout, Chan_AGCutout, Chan_ABCutout);
static const ChannelSet  Mask_ZCutout (Chan_ZCutout);
static const ChannelSet  Mask_Cutouts (Chan_ACutout, Chan_ARCutout, Chan_AGCutout, Chan_ABCutout, Chan_ZCutout);
//
static const ChannelSet  Mask_UvS (Chan_UvS);
static const ChannelSet  Mask_UvT (Chan_UvT);
static const ChannelSet  Mask_UvP (Chan_UvP);
static const ChannelSet  Mask_UvQ (Chan_UvQ);
static const ChannelSet  Mask_Uv2 (Chan_UvS, Chan_UvT);
static const ChannelSet  Mask_Uv3 (Chan_UvS, Chan_UvT, Chan_UvP);
static const ChannelSet  Mask_Uv4 (Chan_UvS, Chan_UvT, Chan_UvP, Chan_UvQ);
//
static const ChannelSet  Mask_ID (Chan_ID0);
static const ChannelSet  Mask_ID2 (Chan_ID0, Chan_ID1);
static const ChannelSet  Mask_ID3 (Chan_ID0, Chan_ID1, Chan_ID2);
static const ChannelSet  Mask_ID4 (Chan_ID0, Chan_ID1, Chan_ID2, Chan_ID3);
//
static const ChannelSet  Mask_All (Chan_All);  // Special set indicating all channels enabled


//--------------
//!rst:cpp:end::
//--------------

OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_CHANNELDEFS_H
