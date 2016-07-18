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

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER


//---------------------------------------------------------------------------
//
// Predefined indices for recommended standard channel definitions
// See OpenEXR TechnicalIntroduction.pdf, pages 19-20
//
//---------------------------------------------------------------------------

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
static const ChannelIdx  Chan_Z             = 11; // Distance to sample
static const ChannelIdx  Chan_ZFront        = 11; // Same as Z for the moment (could be separate to be sampled differently...)
static const ChannelIdx  Chan_ZBack         = 13; // Distance of the back of a (deep) sample from the viewer
//
static const ChannelIdx  Chan_DeepFlags     = 14; // Per-sample DeepFlags
static const ChannelIdx  Chan_SpBits1       = 15; // Subpixel bitmask channel 1 - req for 4x4 mask
static const ChannelIdx  Chan_SpBits2       = 16; // Subpixel bitmask channel 2 - 1-2 req for 8x8 mask
//static const ChannelIdx  Chan_SpBits3       = 17; //DEPRECATED - Subpixel bitmask channel 3 - 1-8 req for 16x16 mask
//static const ChannelIdx  Chan_SpBits4       = 18; //DEPRECATED - Subpixel bitmask channel 4 - 1-8 req for 16x16 mask
//static const ChannelIdx  Chan_SpBits5       = 19; //DEPRECATED - Subpixel bitmask channel 5 - 1-8 req for 16x16 mask
//static const ChannelIdx  Chan_SpBits6       = 20; //DEPRECATED - Subpixel bitmask channel 6 - 1-8 req for 16x16 mask
//static const ChannelIdx  Chan_SpBits7       = 21; //DEPRECATED - Subpixel bitmask channel 7 - 1-8 req for 16x16 mask
//static const ChannelIdx  Chan_SpBits8       = 22; //DEPRECATED - Subpixel bitmask channel 8 - 1-8 req for 16x16 mask
static const ChannelIdx  Chan_SpBitsLast    = Chan_SpBits2; // Last Subpixel bitmask channel
//
static const ChannelIdx  Chan_UvS           = 17; // Texture coordinate S
static const ChannelIdx  Chan_UvT           = 18; // Texture coordinate T
static const ChannelIdx  Chan_UvP           = 19; // Texture coordinate P
static const ChannelIdx  Chan_UvQ           = 20; // Texture coordinate Q
//
static const ChannelIdx  Chan_ID0           = 21; // Numerical identifier for an object/surface
static const ChannelIdx  Chan_ID1           = 22; // Numerical identifier for an object/surface
static const ChannelIdx  Chan_ID2           = 23; // Numerical identifier for an object/surface
static const ChannelIdx  Chan_ID3           = 24; // Numerical identifier for an object/surface
//
static const ChannelIdx  Chan_CutoutA       = 25; // Alpha with cutouts applied (holes)
static const ChannelIdx  Chan_CutoutAR      = 26; // Red-opacity with cutouts applied (holes)
static const ChannelIdx  Chan_CutoutAG      = 27; // Green-opacity with cutouts applied (holes)
static const ChannelIdx  Chan_CutoutAB      = 28; // Blue-opacity with cutouts applied (holes)
static const ChannelIdx  Chan_CutoutZ       = 29; // Z depth with cutouts applied (holes)
//
// Arbitrary channel indices start at this one:
static const ChannelIdx  Chan_ArbitraryStart = Chan_CutoutZ+1;


//---------------------------------------------------------------------------
//
// Predefined ChannelSets(masks) for recommended standard channel definitions
// See OpenEXR TechnicalIntroduction.pdf, pages 19-20
//
//---------------------------------------------------------------------------

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
//
static const ChannelSet  Mask_Y (Chan_Y);
static const ChannelSet  Mask_RY (Chan_RY);
static const ChannelSet  Mask_BY (Chan_BY);
static const ChannelSet  Mask_Yuv (Chan_Y, Chan_RY, Chan_BY);
//
static const ChannelSet  Mask_Z (Chan_Z);
static const ChannelSet  Mask_ZFront (Chan_ZFront);
static const ChannelSet  Mask_ZBack (Chan_ZBack);
static const ChannelSet  Mask_Deep (Chan_ZFront, Chan_ZBack);
static const ChannelSet  Mask_Depth (Chan_Z, Chan_ZFront, Chan_ZBack);
//
static const ChannelSet  Mask_DeepFlags (Chan_DeepFlags);
static const ChannelSet  Mask_SpBits1 (Chan_SpBits1);
static const ChannelSet  Mask_SpBits2 (Chan_SpBits2);
static const ChannelSet  Mask_SpMask4 (Chan_SpBits1); // TODO: deprecate - no need for 4x4 masks anymore
static const ChannelSet  Mask_SpMask8 (Chan_SpBits1, Chan_SpBits2);
//static const ChannelSet  Mask_SpMask16 (Chan_SpBits1, Chan_SpBits2, Chan_SpBits3, Chan_SpBits4,
//                                        Chan_SpBits5, Chan_SpBits6, Chan_SpBits7, Chan_SpBits8); // DEPRECATED
static const ChannelSet  Mask_DeepMetadata (Chan_DeepFlags, Chan_SpBits1, Chan_SpBits2);
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
static const ChannelSet  Mask_CutoutA (Chan_CutoutA);
static const ChannelSet  Mask_CutoutAR (Chan_CutoutAR);
static const ChannelSet  Mask_CutoutAG (Chan_CutoutAG);
static const ChannelSet  Mask_CutoutAB (Chan_CutoutAB);
static const ChannelSet  Mask_CutoutAlpha (Chan_CutoutA, Chan_CutoutAR, Chan_CutoutAG, Chan_CutoutAB);
static const ChannelSet  Mask_CutoutZ (Chan_CutoutZ);
static const ChannelSet  Mask_Cutouts (Chan_CutoutA, Chan_CutoutAR, Chan_CutoutAG, Chan_CutoutAB, Chan_CutoutZ);
//
static const ChannelSet  Mask_All (Chan_All);  // Special set indicating all channels enabled (TODO: needed anymore?)


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_CHANNELDEFS_H
