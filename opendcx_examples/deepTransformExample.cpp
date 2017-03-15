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

#include <OpenDCX/DcxChannelContext.h>
#include <OpenDCX/DcxDeepImageTile.h>
#include <OpenDCX/DcxDeepTransform.h>

#include <stdlib.h>
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
                "  -f <mode>                resample mask mode ('nearest', 'box') (default=box)\n"
                "  -ss <int>                mask super-sampling factor (default=4)\n"
                "\n"
                "Transform options:\n"
                "  -t <x> <y>               translate            (default=0.0,0.0)\n"
                "  -s <x> <y>               scale                (default=1.0,1.0)\n"
                "  -r <deg>                 rotation in degrees  (default=0.0)\n"
                "  -c <x> <y>               rot/scale center     (default=0.0,0.0)\n"
                "  -m <mode>                pixel-loop order 'bwd' or 'fwd' (default=bwd)\n"
                "  -incrop  <x> <y> <r> <t> input crop region\n"
                "  -outcrop <x> <y> <r> <t> output crop region\n"
                "\n"
                "Info options:\n"
                "  -ininfo  <x> <y>         print input deep pixel info\n"
                "  -outinfo <x> <y>         print output deep pixel info\n"
                "\n"
                //"  -flat                    flatten output\n"
                "  -v                       print additional info\n"
                "  -h                       prints this message\n";

         std::cerr << std::endl;
    }
    exit (1);
}


int
main (int argc, char *argv[])
{
    const char* inFile = 0;
    const char* outFile = 0;

    Dcx::DeepTransform::SuperSamplingMode superSampling = Dcx::DeepTransform::SS_RATE_4;
    Dcx::DeepTransform::FilterMode filterMode = Dcx::DeepTransform::FILTER_BOX;
    float translateX = 0.0f;
    float translateY = 0.0f;
    float scaleX     = 1.0f;
    float scaleY     = 1.0f;
    float rotateZ    = 0.0f;
    float centerX    = -INFINITYf; // calc center from image
    float centerY    = -INFINITYf;
    int   incrop[4];   incrop[0] =  incrop[1] = 100000;  incrop[2] =  incrop[3] = -100000;
    int   outcrop[4]; outcrop[0] = outcrop[1] = 100000; outcrop[2] = outcrop[3] = -100000;
    int   loopOrder  = 0; // backwards
    int   infoInX    = -100000;
    int   infoInY    = -100000;
    int   infoOutX   = -100000;
    int   infoOutY   = -100000;
    //bool  flatten    = false;
    //
    bool  verbose = false;

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
            superSampling = (Dcx::DeepTransform::SuperSamplingMode)int(floor(strtod(argv[i + 1], NULL)));
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
            translateX = strtod(argv[i + 1], NULL);
            translateY = strtod(argv[i + 2], NULL);
            i += 3;
        }
        else if (!strcmp(argv[i], "-s"))
        {
            // xy scale:
            if (i > argc - 3)
                usageMessage(argv[0]);
            scaleX = strtod(argv[i + 1], NULL);
            scaleY = strtod(argv[i + 2], NULL);
            i += 3;
        }
        else if (!strcmp(argv[i], "-r"))
        {
            // z rotation:
            if (i > argc - 2)
                usageMessage(argv[0]);
            rotateZ = strtod(argv[i + 1], NULL);
            i += 2;
        }
        else if (!strcmp(argv[i], "-c"))
        {
            // rot/scale xy center:
            if (i > argc - 3)
                usageMessage(argv[0]);
            centerX = strtod(argv[i + 1], NULL);
            centerY = strtod(argv[i + 2], NULL);
            i += 3;
        }
        else if (!strcmp(argv[i], "-m"))
        {
            // Pixel-loop order mode:
            if (i > argc - 2)
                usageMessage(argv[0]);
            if (strcmp(argv[i + 1], "fwd")==0)
                loopOrder = 1;
            i += 2;
        }
        else if (!strcmp(argv[i], "-incrop"))
        {
            // input crop region:
            if (i > argc - 5)
                usageMessage(argv[0]);
            incrop[0] = (int)strtol(argv[i + 1], 0, 0);
            incrop[1] = (int)strtol(argv[i + 2], 0, 0);
            incrop[2] = (int)strtol(argv[i + 3], 0, 0);
            incrop[3] = (int)strtol(argv[i + 4], 0, 0);
            i += 5;
        }
        else if (!strcmp(argv[i], "-outcrop"))
        {
            // Output crop region:
            if (i > argc - 5)
                usageMessage(argv[0]);
            outcrop[0] = (int)strtol(argv[i + 1], 0, 0);
            outcrop[1] = (int)strtol(argv[i + 2], 0, 0);
            outcrop[2] = (int)strtol(argv[i + 3], 0, 0);
            outcrop[3] = (int)strtol(argv[i + 4], 0, 0);
            i += 5;
        }
        else if (!strcmp(argv[i], "-info") || !strcmp(argv[i], "-outinfo"))
        {
            // print info for deep pixel xy:
            if (i > argc - 3)
                usageMessage(argv[0]);
            infoOutX = (int)strtol(argv[i + 1], 0, 0);
            infoOutY = (int)strtol(argv[i + 2], 0, 0);
            i += 3;
        }
        else if (!strcmp(argv[i], "-ininfo"))
        {
            // print info for deep pixel xy:
            if (i > argc - 3)
                usageMessage(argv[0]);
            infoInX = (int)strtol(argv[i + 1], 0, 0);
            infoInY = (int)strtol(argv[i + 2], 0, 0);
            i += 3;
        }
        //else if (!strcmp(argv[i], "-flat"))
        //{
        //    // Flatten output:
        //    if (i > argc - 1)
        //        usageMessage(argv[0]);
        //    flatten = true;
        //    i += 1;
        //}
        else if (!strcmp(argv[i], "-v"))
        {
            // Verbose mode:
            if (i > argc - 1)
                usageMessage(argv[0]);
            verbose = true;
            i += 1;
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


    Dcx::ChannelContext chanCtx; // stores shared channel aliases

    try
    {
        OPENEXR_IMF_NAMESPACE::Header inHeader; // for access to displayWindow...
        OPENEXR_IMF_NAMESPACE::DeepImage inDeepImage;
        OPENEXR_IMF_NAMESPACE::loadDeepScanLineImage(std::string(inFile), inHeader, inDeepImage);

        // Dcx::DeepTile stores the ChannelSet along with the channel ptrs:
        Dcx::DeepImageInputTile inDeepTile(inHeader, inDeepImage, chanCtx, true/*Yup*/);

        IMATH_NAMESPACE::Box2i in_bbox = inDeepTile.dataWindow();
        if (incrop[0] <= incrop[2] && incrop[1] <= incrop[3])
        {
            in_bbox.min.x = std::max(in_bbox.min.x, incrop[0]);
            in_bbox.min.y = std::max(in_bbox.min.y, incrop[1]);
            in_bbox.max.x = std::max(in_bbox.min.x, std::min(incrop[2], in_bbox.max.x));
            in_bbox.max.y = std::max(in_bbox.min.y, std::min(incrop[3], in_bbox.max.y));
        }

        if (verbose)
        {
            std::cout << "reading file '" << inFile << std::endl;
            std::cout << "  in bbox" << in_bbox;
            inDeepTile.channels().print(", in channels=", std::cout, &chanCtx);
            std::cout << std::endl;
        }

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

        IMATH_NAMESPACE::Box2i out_bbox = xform.transform(in_bbox, 0/*clamp_to*/);
        if (outcrop[0] <= outcrop[2] && outcrop[1] <= outcrop[3])
        {
            out_bbox.min.x = std::max(out_bbox.min.x, outcrop[0]);
            out_bbox.min.y = std::max(out_bbox.min.y, outcrop[1]);
            out_bbox.max.x = std::max(out_bbox.min.x, std::min(outcrop[2], out_bbox.max.x));
            out_bbox.max.y = std::max(out_bbox.min.y, std::min(outcrop[3], out_bbox.max.y));
        }
        // Pad output bbox by one pixel:
        const int pad = 1;//(black_outside)?1:0;
        out_bbox.min.x -= pad; out_bbox.min.y -= pad;
        out_bbox.max.x += pad; out_bbox.max.y += pad;

        Dcx::ChannelSet out_channels = inDeepTile.channels();
        out_channels += Dcx::Mask_DeepMetadata;

        // DeepTransform will add spmask/flags channels so make sure we're writing them:
        Dcx::DeepImageOutputTile outDeepTile(inDeepTile, out_bbox, true/*Yup*/);
        outDeepTile.setChannels(out_channels);
        outDeepTile.setOutputFile(outFile, OPENEXR_IMF_NAMESPACE::INCREASING_Y/*lineOrder*/);
        if (verbose) {
            std::cout << "  transform:" << std::endl << std::fixed << xform.matrix();
            std::cout << "  out file '" << outFile << std::endl;
            std::cout << "  out_bbox" << out_bbox;
            outDeepTile.channels().print(", out channels=", std::cout, &chanCtx);
            std::cout << std::endl;
        }

        Dcx::ChannelSet xform_channels = inDeepTile.channels();

        // This DeepPixel get reused:
        Dcx::DeepPixel deepPixel(xform_channels);

        if (loopOrder == 1)
        {
            // Do forward-transform which is less efficient as the entire
            // output tile must be kept cached until all input pixels are
            // finished:
            if (verbose)
                std::cout << "    DO FORWARD-TRANSFORM LOOP" << std::endl;
            for (int inY=in_bbox.min.y; inY <= in_bbox.max.y; ++inY)
            {
                for (int inX=in_bbox.min.x; inX <= in_bbox.max.x; ++inX)
                {

                    // Transform the input deep pixel to the output tile:
                    xform.transformPixel(inDeepTile, inX, inY, outDeepTile);

                    if (inX==infoInX && inY==infoInY)
                    {
                        std::cout << "in[" << infoOutX << ", " << infoOutY << "]";
                        deepPixel.printInfo(std::cout, "=");
                    }

                }
            }
            outDeepTile.writeTile(true/*flush*/);

        }
        else
        {
            // Do reverse-transform which is more memory-efficient as each output
            // scanline can be flushed as it's completed:
            if (verbose)
                std::cout << "    DO BACKWARDS-TRANSFORM LOOP" << std::endl;
            if (inDeepTile.isActivePixel(infoInX, infoInY))
            {
                std::cout << "in[" << infoInX << ", " << infoInY << "]";
                inDeepTile.getDeepPixel(infoInX, infoInY, deepPixel);
                deepPixel.printInfo(std::cout, "=");
            }

            for (int outY=out_bbox.min.y; outY <= out_bbox.max.y; ++outY)
            {
                for (int outX=out_bbox.min.x; outX <= out_bbox.max.x; ++outX)
                {
                    // Sample the output deep pixel from the input tile:
                    xform.sample(outX, outY, inDeepTile, xform_channels, deepPixel);

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

        }

        // If all lines are flushed on write the tile should be using zero bytes
        // by end:
        if (verbose)
            std::cout << "  out_tile bytes remaining (should be 0)=" << outDeepTile.bytesUsed() << std::endl;

    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exitStatus = 1;
    }

    return exitStatus;
}
