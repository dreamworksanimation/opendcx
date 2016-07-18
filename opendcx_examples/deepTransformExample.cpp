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
/// @file deepTransformExample.cpp


//
//  deepTransformExample
//
//      An example use of OpenDCX to read a deep exr, transform it in 2D
//      and write out to another deep exr.
//
//      Leverages the Dcx::DeepTransform class' subpixel mask resampling
//      functionality to allow fractional pixel transforms to be applied
//      without losing subpixel masks or flags metadata.
//
//      Speed & threading:
//      This example routine is single-threaded, but it's relatively easy
//      to spawn multiple threads to work on separate scanlines which is
//      safe as long as the Dcx::ChannelContext is shared between them.
//

#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfDeepImageIO.h>

#include <OpenDCX/DcxDeepImageTile.h>
#include <OpenDCX/DcxDeepTransform.h>

#include <iostream>
#include <exception>

void
usageMessage (const char argv0[], bool verbose=false)
{
    std::cerr << "usage: " << argv0 << " [options] infile outfile" << std::endl;

    if (verbose)
    {
        std::cerr << "\n"
                "Applies a 2D transform to an OpenEXR deep image\n"
                "Reads the input file line by line applying the 2D transform and writing to output file.\n"
                "Example use of the OpenDCX DeepTransform class and the resampling of subpixel masks to\n"
                "provide fractional-pixel filtering while preserving the subpixel mask functionality.\n"
                "\n"
                "Mask resampling options:\n"
                "  -f <mode>  resample mask mode ('nearest', 'box') (default=box)\n"
                "  -ss <int>  mask super-sampling factor (default=4)\n"
                "\n"
                "Transform options:\n"
                "  -t <x> <y> translate            (default=0.0,0.0)\n"
                "  -s <x> <y> scale                (default=1.0,1.0)\n"
                "  -r <deg>   rotation in degrees  (default=0.0)\n"
                "  -c <x> <y> rot/scale center     (default=0.0,0.0)\n"
                "\n"
                "Info options:\n"
                "  -ininfo  <x> <y>  print input deep pixel info\n"
                "  -outinfo <x> <y>  print output deep pixel info\n"
                "\n"
                "  -h         prints this message\n";

         std::cerr << std::endl;
    }
    exit (1);
}


int
main (int argc, char *argv[])
{
    const char* inFile = 0;
    const char* outFile = 0;

    int   superSampling = 4;
    Dcx::DeepTransform::FilterMode filterMode = Dcx::DeepTransform::FILTER_BOX;
    float translateX = 0.0f;
    float translateY = 0.0f;
    float scaleX     = 1.0f;
    float scaleY     = 1.0f;
    float rotateZ    = 0.0f;
    float centerX    = -INFINITYf; // calc center from image
    float centerY    = -INFINITYf;
    int   infoInX    = -100000;
    int   infoInY    = -100000;
    int   infoOutX   = -100000;
    int   infoOutY   = -100000;

    //
    // Parse the command line.
    //

    if (argc < 2)
        usageMessage(argv[0], true);

    int i = 1;
    while (i < argc)
    {
        if (!strcmp(argv[i], "-ss"))
        {
            // Subpixel mask resample mode:
            if (i > argc - 2)
                usageMessage(argv[0]);
            superSampling = (int)floor(strtol(argv[i + 1], 0, 0));
            i += 2;
        }
        if (!strcmp(argv[i], "-f"))
        {
            // Subpixel mask resample mode:
            if (i > argc - 2)
                usageMessage(argv[0]);
            if (!strcmp(argv[i + 1], "box"))
                filterMode = Dcx::DeepTransform::FILTER_BOX;
            else if (!strcmp(argv[i + 1], "nearest"))
                filterMode = Dcx::DeepTransform::FILTER_NEAREST;
            i += 2;
        }
        else if (!strcmp(argv[i], "-t"))
        {
            // xy translation:
            if (i > argc - 3)
                usageMessage(argv[0]);
            translateX = strtol(argv[i + 1], 0, 0);
            translateY = strtol(argv[i + 2], 0, 0);
            i += 3;
        }
        else if (!strcmp(argv[i], "-s"))
        {
            // xy scale:
            if (i > argc - 3)
                usageMessage(argv[0]);
            scaleX = strtol(argv[i + 1], 0, 0);
            scaleY = strtol(argv[i + 2], 0, 0);
            i += 3;
        }
        else if (!strcmp(argv[i], "-r"))
        {
            // z rotation:
            if (i > argc - 2)
                usageMessage(argv[0]);
            rotateZ = strtol(argv[i + 1], 0, 0);
            i += 2;
        }
        else if (!strcmp(argv[i], "-c"))
        {
            // rot/scale xy center:
            if (i > argc - 3)
                usageMessage(argv[0]);
            centerX = strtol(argv[i + 1], 0, 0);
            centerY = strtol(argv[i + 2], 0, 0);
            i += 3;
        }
        else if (!strcmp(argv[i], "-info") || !strcmp(argv[i], "-outinfo"))
        {
            // print info for deep pixel xy:
            if (i > argc - 3)
                usageMessage(argv[0]);
            infoOutX = (int)floor(strtol(argv[i + 1], 0, 0));
            infoOutY = (int)floor(strtol(argv[i + 2], 0, 0));
            i += 3;
        }
        else if (!strcmp(argv[i], "-ininfo"))
        {
            // print info for deep pixel xy:
            if (i > argc - 3)
                usageMessage(argv[0]);
            infoInX = (int)floor(strtol(argv[i + 1], 0, 0));
            infoInY = (int)floor(strtol(argv[i + 2], 0, 0));
            i += 3;
        }
        else if (!strcmp(argv[i], "-h"))
        {
            // Print help message:
            usageMessage(argv[0], true);
        }
        else
        {
            // Image file names:
            if (inFile == 0)
                inFile = argv[i];
            else
                outFile = argv[i];
            i += 1;
        }
    }

    if (inFile == 0 || outFile == 0)
        usageMessage(argv[0]);

    //
    // Load inFile, transform each scanline, and save the result in outFile.
    //

    int exitStatus = 0;


    Dcx::ChannelContext channelCtx; // stores shared channel aliases

    try
    {
        Imf::Header inHeader; // for access to displayWindow...
        Imf::DeepImage inDeepImage;
        Imf::loadDeepScanLineImage(std::string(inFile), inHeader, inDeepImage);

        // Dcx::DeepTile stores the ChannelSet along with the channel ptrs:
        Dcx::DeepImageInputTile inDeepTile(inHeader, inDeepImage, channelCtx, true/*Yup*/);

#ifdef DCX_DEBUG_TRANSFORM
std::cout << "reading file '" << inFile << "'" << std::endl;
std::cout << " in_bbox" << inDeepTile.dataWindow() << std::endl;
inDeepTile.channels().print("channels=", std::cout, channelCtx); std::cout << std::endl;
#endif

        // Output tile is copy of in tile:
        Dcx::DeepImageOutputTile outDeepTile(inDeepTile);
        outDeepTile.setOutputFile(outFile, Imf::INCREASING_Y/*lineOrder*/);

        if (centerX <= -INFINITYf)
        {
            centerX = inDeepTile.w()/2.0f;
            centerY = inDeepTile.h()/2.0f;
        }

        Dcx::DeepTransform xform(superSampling, filterMode);
        xform.translate(centerX, centerY);
        xform.rotate(radians(rotateZ));
        xform.scale(scaleX, scaleY);
        xform.translate(-centerX, -centerY);
        xform.translate(translateX, translateY);

        Dcx::ChannelSet xform_channels = inDeepTile.channels();

        // This DeepPixel get reused:
        Dcx::DeepPixel deepPixel(xform_channels);

        if (inDeepTile.isActivePixel(infoInX, infoInY))
        {
            std::cout << "in[" << infoInX << ", " << infoInY << "]";
            inDeepTile.getDeepPixel(infoInX, infoInY, deepPixel);
            deepPixel.printInfo(std::cout, "=");
        }

        Imath::Box2i out_bbox = xform.transform(inDeepTile.dataWindow(), 0/*&in_bbox*//*clamp_to*/);
#ifdef DCX_DEBUG_TRANSFORM
std::cout << "out_bbox" << out_bbox << std::endl;
#endif

        for (int outY=out_bbox.min.y; outY <= out_bbox.max.y; ++outY)
        {
            for (int outX=out_bbox.min.x; outX <= out_bbox.max.x; ++outX)
            {
                // Sample the output deep pixel from the input tile:
                xform.sample(outX, outY, inDeepTile, deepPixel);

                if (outX==infoOutX && outY==infoOutY)
                {
                    std::cout << "out[" << infoOutX << ", " << infoOutY << "]";
                    deepPixel.printInfo(std::cout, "=");
                }

                // Save deep pixel in output tile:
                outDeepTile.setDeepPixel(outX, outY, deepPixel);

            }

            outDeepTile.writeScanline(outY, true/*flush*/);
        }
#ifdef DCX_DEBUG_TRANSFORM
std::cout << "  out_tile bytes=" << outDeepTile.bytesUsed() << std::endl;
#endif

    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exitStatus = 1;
    }

    return exitStatus;
}
