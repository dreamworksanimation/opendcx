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
/// @file readDeepPixelExample.cpp


//
//  readDeepPixelExample
//
//      Simple reader that uses the DeepTile & DeepPixel functions to
//      get and print DeepSegment info for deep pixels.
//


#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfDeepImageIO.h>

#include <OpenDCX/DcxChannelContext.h>
#include <OpenDCX/DcxDeepImageTile.h>

#include <stdlib.h>

void
usageMessage (const char argv0[], bool verbose=false)
{
    std::cerr << "usage: " << argv0 << " [options] infile" << std::endl;

    if (verbose)
    {
        std::cerr << "\n"
                "Print info about a deep pixel or line of deep pixels\n"
                "\n"
                "Options:\n"
                "  -pixel <x> <y>  print a single deep pixel\n"
                "  -line <y>       print all deep pixels on line\n"
                "  -nomask         don't print subpixel mask patterns\n"
                "\n"
                "  -h              prints this message\n";

         std::cerr << std::endl;
    }
    exit (1);
}


int
main (int argc, char *argv[])
{
    const char* inFile = 0;
    int infoX = -100000;
    int infoY = -100000;
    bool showMasks = true;
    bool verbose = false;

    //
    // Parse the command line.
    //

    if (argc < 2)
        usageMessage(argv[0], true);

    int i = 1;
    while (i < argc)
    {
        if (!strcmp(argv[i], "-pixel"))
        {
            // print info for deep pixel xy:
            if (i > argc - 3)
                usageMessage(argv[0]);
            infoX = (int)floor(strtol(argv[i + 1], 0, 0));
            infoY = (int)floor(strtol(argv[i + 2], 0, 0));
            i += 3;
        }
        else if (!strcmp(argv[i], "-line"))
        {
            // print info for deep pixels on line y:
            if (i > argc - 2)
                usageMessage(argv[0]);
            infoY = (int)floor(strtol(argv[i + 1], 0, 0));
            i += 2;
        }
        else if (!strcmp(argv[i], "-nomask"))
        {
            // print info for deep pixels on line y:
            if (i > argc - 1)
                usageMessage(argv[0]);
            showMasks = false;
            i += 1;
        }
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
            // Image file name:
            if (inFile == 0)
                inFile = argv[i];
            i += 1;
        }
    }

    if (inFile == 0)
        usageMessage(argv[0]);

    int exitStatus = 0;

    Dcx::ChannelContext chanCtx; // stores shared channel aliases

    try
    {
        Imf::Header inHeader; // for access to displayWindow...
        Imf::DeepImage inDeepImage;
        Imf::loadDeepScanLineImage(std::string(inFile), inHeader, inDeepImage);

        // Dcx::DeepTile stores the ChannelSet along with the channel ptrs:
        Dcx::DeepImageInputTile inDeepTile(inHeader, inDeepImage, chanCtx, true/*Yup*/);
        if (verbose)
        {
            std::cout << "reading file '" << inFile << "'" << std::endl;
            std::cout << "  in bbox" << inDeepTile.dataWindow() << std::endl;
            inDeepTile.channels().print("  in channels=", std::cout, &chanCtx); std::cout << std::endl;
        }

        // Dcx::DeepPixel stores all the DeepSegments (samples) for the entire deep pixel:
        Dcx::DeepPixel deepPixel(inDeepTile.channels());

        // Just get one pixel:
        if (inDeepTile.isActivePixel(infoX, infoY))
        {
            std::cout << "[" << infoX << ", " << infoY << "]";
            inDeepTile.getDeepPixel(infoX, infoY, deepPixel);
            deepPixel.printInfo(std::cout,
                                "="/*prefix*/,
                                2/*padding*/,
                                showMasks);
        }

        // Print an entire line's worth:
        if (infoX == -100000 && infoY >= inDeepTile.y() && infoY <= inDeepTile.t()) {
            // Loop example going through entire input tile:
            for (infoX=inDeepTile.x(); infoX <= inDeepTile.r(); ++infoX)
            {
                std::cout << "-----------------------------------------------------------------------------" << std::endl;
                inDeepTile.getDeepPixel(infoX, infoY, deepPixel);
                std::cout << "[" << infoX << ", " << infoY << "]";
                deepPixel.printInfo(std::cout,
                                    "=",
                                    2/*padding*/,
                                    showMasks);
            }
        }

    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exitStatus = 1;
    }

    return exitStatus;
}
