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
/// @file raySampleAccumulatorExample.cpp


//
//  raySampleAccumulatorExample
//
//      Example that converts input deep pixels into sphere primitives
//      which are ray traced multiple times at each output pixel producing
//      deep pixels with subpixel masks.
//
//      A simple example of collapsing (accumulating) subpixel ray
//      intersections together to form a single output deep sample with
//      an accumulated subpixel mask.
//


#include <OpenEXR/ImfHeader.h>
#include <OpenEXR/ImfDeepImageIO.h>
#include <OpenEXR/ImathMatrix.h>

#include <OpenDCX/DcxChannelContext.h>
#include <OpenDCX/DcxDeepImageTile.h>
#include <OpenDCX/DcxDeepTransform.h>  // for radians()

#include <stdlib.h>

#ifdef DEBUG
#  include <assert.h>
#endif

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------


typedef uint64_t SurfaceID;


struct MyRay
{
    Imath::V3f  origin;
    Imath::V3f  dir;
};

struct MyCamera
{
    Imath::M44f matrix;
    float       lens;                       // lens magnification
    float       fX, fY, fR, fT, fW, fH;     // float extents of viewport
    float       fWinAspect;                 // aspect ratio of viewport

    MyCamera(const Imath::V3f& translate,
             const Imath::V3f& rotate_degrees,
             float focalLength,
             float hAperture,
             const Imath::Box2i& format,
             float pixel_aspect) :
        fX(format.min.x),
        fY(format.min.y),
        fR(format.max.x),
        fT(format.max.y),
        fW(format.max.x - format.min.x + 1),
        fH(format.max.y - format.min.y + 1)
    {
        matrix.makeIdentity();
        matrix.rotate(Imath::V3f(radians(rotate_degrees.x),
                                 radians(rotate_degrees.y),
                                 radians(rotate_degrees.z)));
        matrix.translate(translate);
        lens = hAperture / focalLength;
        fWinAspect = (fH / fW)/pixel_aspect; // Image aspect with pixel-aspect mixed in
    }

    inline
    void getNdcCoord(float pixelX, float pixelY,
                     float& u, float& v) const
    {
        u = (pixelX - fX)/fW*2.0f - 1.0f;
        v = (pixelY - fY)/fH*2.0f - 1.0f;
    }

    inline
    void buildRay(float pixelX, float pixelY, MyRay& R) const
    {
        float u, v;
        getNdcCoord(pixelX, pixelY, u, v);
        //
        R.origin = matrix.translation();
        matrix.multDirMatrix(Imath::V3f(u*lens*0.5f, v*lens*0.5f*fWinAspect, -1.0f), R.dir);
        R.dir.normalize();
    }
};

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

struct MySphere
{
    Imath::V3f  center;
    float       radius;
    Imath::V4f  color;
    SurfaceID   surfID;

    inline
    bool intersect(const MyRay& R,
                   double& tmin,
                   double& tmax,
                   Imath::V3f& P,
                   Imath::V3f& N) const
    {
        const Imath::V3f s_minus_r = R.origin - center;
        const double a = R.dir.length2();
        const double b = 2.0 * R.dir.dot(s_minus_r);
        const double c = s_minus_r.length2() - radius*radius;
        const double discrm = b*b - 4.0*a*c;
        if (discrm >= EPSILONd) {
           const double l = sqrt(discrm);
           tmin = (-b - l) / (2.0 * a);
           tmax = (-b + l) / (2.0 * a);
           if (tmin < EPSILONd && tmax < EPSILONd)
              return false; // behind sphere
           P = R.origin + R.dir*tmin;
           N = P - center;
           N.normalize();
           return true;
        }
        if (fabs(discrm) < EPSILONd) {
           // Ray is tangent to sphere:
           tmin = tmax = -b / (2.0 * a);
           if (tmin < EPSILONd)
              return false; // behind sphere
           P = R.origin + R.dir*tmin;
           N = P - center;
           N.normalize();
           return true;
        }
        return false;
    }

};

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------


enum PrimType
{
    PRIMTYPE_SPHERE         = 0,    // Only support spheres in this example
    //PRIMTYPE_DISC           = 1,
    //PRIMTYPE_TRIANGLE       = 2
};


struct DeepIntersection
{
    double          tmin;           // Distance from ray origin to nearest intersection point
    double          tmax;           // Distance from ray origin to farthest intersection point
    void*           primPtr;        // Pointer to hit primitive
    PrimType        primType;       // Object type to cast primPtr
    Imath::V4f      color;          // Shaded surface color at intersection
    Imath::V3f      N;              // Shaded surface normal
    Dcx::SpMask8    spmask;         // Subpixel mask
    int             count;          // Number of intersections combined with this one
};

//! List of DeepIntersections
typedef std::vector<DeepIntersection> DeepIntersectionList;

//
// A surface can overlap itself causing the same surface ID to show up multiple times
// in the same deep intersection list, but we don't want to always combine them if the
// surface intersections are facing away from each other or are not close in Z.
//

// List of same-surface DeepIntersection indices
typedef std::vector<size_t> DeepSurfaceIntersectionList;
typedef std::map<SurfaceID, DeepSurfaceIntersectionList> DeepSurfaceIntersectionMap;

//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

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
                "  -skip <n>       read every nth input pixel when creating spheres (default 8)\n"
                "  -scale <v>      globally scale of sphere radius (default 40.0)\n"
                "\n"
                "  -sp <v>         xy subpixel sampling rate (default 16)\n"
                "  -spX <x>        x subpixel sampling rate (default 16)\n"
                "  -spY <y>        y subpixel sampling rate (default 16)\n"
                "  -zthresh <v>    z-distance threshold for combining samples\n"
                "\n"
                "  -camt <x><y><z> camera translation (default input image w/2,h/2,w)\n"
                "  -camr <x><y><z> camera rotation (default 0,0,0)\n"
                "  -camfl <v>      camera focal-length (default 50.0)\n"
                "  -camha <v>      camera horizontal aperture (default 24.0)\n"
                "\n"
                "  -v              print additional info\n"
                "  -h              prints this message\n";

         std::cerr << std::endl;
    }
    exit (1);
}


int
main (int argc, char *argv[])
{

    const char* inFile = 0;
    const char* outFile = 0;

    int   skipPixel = 8;
    float scaleSpheres = 40.0f;
    //
    int   subpixelXRate = 16;
    int   subpixelYRate = 16;
    float deepCombineZThreshold = 1.0f;
    //
    bool  centerCam = true;
    float camTx=0.0f, camTy=0.0f, camTz=0.0f;
    float camRx=0.0f, camRy=0.0f, camRz=0.0f;
    float camFocal = 50.0f;
    float camHaper = 24.0f;
    //
    bool  verbose = false;

    //
    // Parse the command line.
    //

    if (argc < 2)
        usageMessage(argv[0], true);

    {
        int i = 1;
        while (i < argc)
        {
            if (!strcmp(argv[i], "-skip"))
            {
                // Input pixel-skip rate:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                skipPixel = (int)floor(strtol(argv[i + 1], 0, 0));
                i += 2;
            }
            else if (!strcmp(argv[i], "-scale"))
            {
                // Sphere scale factor:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                scaleSpheres = fabs(strtol(argv[i + 1], 0, 0));
                i += 2;
            }
            else if (!strcmp(argv[i], "-sp"))
            {
                // Subpixel rate:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                subpixelXRate = std::max(1, (int)floor(strtol(argv[i + 1], 0, 0)));
                subpixelYRate = subpixelXRate;
                i += 2;
            }
            else if (!strcmp(argv[i], "-spX"))
            {
                // Subpixel X rate:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                subpixelXRate = std::max(1, (int)floor(strtol(argv[i + 1], 0, 0)));
                i += 2;
            }
            else if (!strcmp(argv[i], "-spY"))
            {
                // Subpixel Y rate:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                subpixelYRate = std::max(1, (int)floor(strtol(argv[i + 1], 0, 0)));
                i += 2;
            }
            else if (!strcmp(argv[i], "-zthresh"))
            {
                // Subpixel rate:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                deepCombineZThreshold = fabs(strtol(argv[i + 1], 0, 0));
                i += 2;
            }
            else if (!strcmp(argv[i], "-camt"))
            {
                // Camera translation:
                if (i > argc - 4)
                    usageMessage(argv[0]);
                camTx = strtol(argv[i + 1], 0, 0);
                camTy = strtol(argv[i + 2], 0, 0);
                camTz = strtol(argv[i + 3], 0, 0);
                centerCam = false;
                i += 4;
            }
            else if (!strcmp(argv[i], "-camr"))
            {
                // Camera rotation:
                if (i > argc - 4)
                    usageMessage(argv[0]);
                camRx = strtol(argv[i + 1], 0, 0);
                camRy = strtol(argv[i + 2], 0, 0);
                camRz = strtol(argv[i + 3], 0, 0);
                i += 4;
            }
            else if (!strcmp(argv[i], "-camfl"))
            {
                // Camera focal-length:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                camFocal = fabs(strtol(argv[i + 1], 0, 0));
                i += 2;
            }
            else if (!strcmp(argv[i], "-camha"))
            {
                // Camera horiz-aperture:
                if (i > argc - 2)
                    usageMessage(argv[0]);
                camHaper = fabs(strtol(argv[i + 1], 0, 0));
                i += 2;
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
                // Image file names:
                if (inFile == 0)
                    inFile = argv[i];
                else
                    outFile = argv[i];
                i += 1;
            }
        }
    }

    if (inFile == 0 || outFile == 0)
        usageMessage(argv[0]);

    //
    // Load inFile, spawn spheres for each input deep sample, raytrace each
    // output pixel and save the result in outFile.
    //

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

        // Output tile is copy of in tile.
        // This is for convenience, the output image can be completely
        // different.
        Dcx::DeepImageOutputTile outDeepTile(inDeepTile);
        outDeepTile.setOutputFile(outFile, Imf::INCREASING_Y/*lineOrder*/);

        // If camera translation not specified, move to output image w/2,h/2,w:
        if (centerCam)
        {
            camTx = outDeepTile.w()/2.0f + 0.5f;
            camTy = outDeepTile.h()/2.0f + 0.5f;
            camTz = float(outDeepTile.w());
        }

        //--------------------------------------------------------------------------

        MyCamera cam(Imath::V3f(camTx, camTy, camTz)/*translate*/,
                     Imath::V3f(camRx, camRy, camRz)/*rotate*/,
                     camFocal/*focalLength*/,
                     camHaper/*hAperture*/,
                     inDeepTile.displayWindow(),
                     1.0f/*pixel_aspect*/);

        // Make a bunch of spheres to intersect:
        std::vector<MySphere> pixelSpheres;

        Dcx::ChannelSet shaderChannels(inDeepTile.channels());
        shaderChannels &= Dcx::Mask_RGBA; // only use rgba

        uint64_t ID = 0;
        int maxSamples = 0;
        Dcx::DeepPixel inDeepPixel(shaderChannels);
        for (int inY=inDeepTile.y(); inY <= inDeepTile.t(); inY += skipPixel)
        {
            for (int inX=inDeepTile.x(); inX <= inDeepTile.r(); inX += skipPixel)
            {
                inDeepTile.getDeepPixel(inX, inY, inDeepPixel);
                const size_t nSamples = inDeepPixel.size();
                for (size_t s=0; s < nSamples; ++s)
                {
                    const Dcx::DeepSegment ds = inDeepPixel.getSegment(s);
                    const Dcx::Pixelf dp = inDeepPixel.getSegmentPixel(s);
                    //
                    MySphere sphere;
                    sphere.center.setValue(float(inX), float(inY), -ds.Zf);
                    sphere.radius = std::min(0.1f, ds.Zb - ds.Zf) * scaleSpheres;
                    int c = 0;
                    foreach_channel(z, shaderChannels)
                        sphere.color[c++] = dp[z];
                    sphere.surfID = ID++;

                    pixelSpheres.push_back(sphere);
                }
                maxSamples = std::max(maxSamples, (int)nSamples);
            }
        }
        if (verbose)
            std::cout << "raytracing " << pixelSpheres.size() << " spheres for " << outDeepTile.h() << " output lines" << std::endl;

        // Output DeepPixel reused at each pixel:
        Dcx::DeepPixel outDeepPixel(shaderChannels);
        outDeepPixel.reserve(10);

        // Temp list of intersections, reused at each subpixel:
        DeepIntersectionList deepIntersectionList;
        deepIntersectionList.reserve(20);

        // The accumulated list of intersections for the whole pixel:
        DeepIntersectionList deepAccumIntersectionList;
        deepAccumIntersectionList.reserve(maxSamples);

        // Map of unique prim intersections:
        DeepSurfaceIntersectionMap deepSurfaceIntersectionMap;

        for (int outY=outDeepTile.y(); outY <= outDeepTile.t(); ++outY)
        {
            if (verbose)
                std::cout << "  line " << outY << std::endl;
            for (int outX=outDeepTile.x(); outX <= outDeepTile.r(); ++outX)
            {
                outDeepPixel.clear();
                deepAccumIntersectionList.clear();
                deepSurfaceIntersectionMap.clear();

                for (int sy=0; sy < subpixelYRate; ++sy)
                {
                    // Subpixel x offset:
                    const float sdy = float(sy)/float(subpixelYRate - 1);

                    for (int sx=0; sx < subpixelXRate; ++sx)
                    {
                        // Subpixel y offset:
                        const float sdx = float(sx)/float(subpixelXRate - 1);

                        // Build output spmask for this subpixel:
                        Dcx::SpMask8 outSpMask = Dcx::SpMask8::allBitsOff;
                        int outSpX, outSpY, outSpR, outSpT;
                        Dcx::SpMask8::mapXCoord(sx, subpixelXRate, outSpX, outSpR);
                        Dcx::SpMask8::mapYCoord(sy, subpixelYRate, outSpY, outSpT);
                        outSpMask.setSubpixels(outSpX, outSpY, outSpR, outSpT);
                        //outSpMask.printPattern(std::cout, "  ");

                        // Build a ray at this subpixel offset.
                        // (this offset would normally include stochastic jitter)
                        MyRay R;
                        cam.buildRay(float(outX)+sdx,
                                     float(outY)+sdy, R);

                        deepIntersectionList.clear();

                        // Naively intersect the ray with all the spheres - obviously
                        // in practice this would use an acceleration structure:
                        const size_t nSpheres = pixelSpheres.size();
                        for (size_t i=0; i < nSpheres; ++i)
                        {
                            const MySphere& sphere = pixelSpheres[i];
                            double tmin, tmax;
                            Imath::V3f P, N;
                            if (sphere.intersect(R, tmin, tmax, P, N))
                            {
                                bool addIt = true;
                                {
                                    //
                                    // Shade step would go here - make spheres shiny, or check
                                    // if alpha < epsilon to produce holes (addIt = false)
                                    //
                                }
                                if (addIt)
                                {
                                    // Add a intersection reference to the sphere:
                                    deepIntersectionList.push_back(DeepIntersection());
                                    DeepIntersection& I = deepIntersectionList[deepIntersectionList.size()-1];
                                    I.tmin     = tmin;
                                    I.tmax     = tmin;
                                    I.primPtr  = (void*)&sphere;
                                    I.primType = PRIMTYPE_SPHERE;
                                    I.color    = sphere.color;
                                    I.N        = N;
                                    I.spmask   = outSpMask;
                                    I.count    = 1;
                                }
                            }
                        }

                        const size_t nIntersections = deepIntersectionList.size();
                        if (nIntersections == 0)
                            continue;

                        // Find all the intersections that should be combined:
                        for (size_t i=0; i < nIntersections; ++i)
                        {
                            DeepIntersection& I = deepIntersectionList[i];
#ifdef DEBUG
                            assert(I.primPtr);
#endif

                            // We only understand spheres in this example...:
                            if (I.primType != PRIMTYPE_SPHERE)
                                continue;
                            const MySphere* sphere = static_cast<MySphere*>(I.primPtr);

                            // Has the sphere's surface ID been intersected before for
                            // this subpixel?
                            DeepSurfaceIntersectionMap::iterator it = deepSurfaceIntersectionMap.find(sphere->surfID);
                            if (it == deepSurfaceIntersectionMap.end())
                            {
                                // Not in map yet, add intersection to the accumulate list:
                                deepAccumIntersectionList.push_back(I);
                                const size_t accumIndex = deepAccumIntersectionList.size()-1;
                                DeepSurfaceIntersectionList dsl;
                                dsl.reserve(10);
                                dsl.push_back(accumIndex);
                                deepSurfaceIntersectionMap[sphere->surfID] = dsl;
                                continue;
                            }

                            //------------------------------------------------------------------------
                            // Intersection already in map, so let's see if it's close enough in Z
                            // and N to combine with one of the previous intersections.
                            //
                            // (TODO: we only check the first match, it's better to find all potential
                            //  matches and select the closest match)
                            //
                            //------------------------------------------------------------------------
                            DeepSurfaceIntersectionList& dsl = it->second;
                            const size_t nCurrentSurfaces = dsl.size();
                            bool match = false;
                            for (size_t j=0; j < nCurrentSurfaces; ++j)
                            {
                                DeepIntersection& matchedI = deepAccumIntersectionList[dsl[j]];
                                //---------------------------------------------------------------------
                                // Compare criteria:
                                //      * minZ within threshold range
                                //      * maxZ within threshold range
                                //      * normal < 180deg diff
                                // (TODO: compare other params like color contrast to retain high-freq
                                //  detail!)
                                //---------------------------------------------------------------------
                                const float eMinZ = matchedI.tmin - deepCombineZThreshold;
                                const float eMaxZ = matchedI.tmax + deepCombineZThreshold;
                                const float eN = I.N.dot(matchedI.N);
                                if (I.tmin < eMinZ || I.tmin > eMaxZ || eN < 0.5f)
                                    continue; // no match, skip to next

                                // Matched! Combine intersections:
                                matchedI.tmin = std::min(matchedI.tmin, I.tmin);
                                matchedI.tmax = std::max(matchedI.tmax, I.tmin);

                                matchedI.color  += I.color; // Add colors together
                                matchedI.spmask |= I.spmask; // Or the subpixel masks together
                                matchedI.count  += 1; // Increase combined count
                                match = true;
                                break;
                            }
                            if (!match)
                            {
                                //--------------------------------------------------------------
                                // No match in current surface list, add intersection as unique.
                                // Copy unique intersection to accumlate list:
                                //--------------------------------------------------------------
                                deepAccumIntersectionList.push_back(I);
                                const size_t accumIndex = deepAccumIntersectionList.size()-1;
                                dsl.push_back(accumIndex);
                            }

                        } // nIntersections loop

                    } // subpixel-x loop

                } // subpixel-y loop

                const size_t nIntersections = deepAccumIntersectionList.size();
                if (nIntersections == 0)
                {
                    outDeepTile.clearDeepPixel(outX, outY);
                    continue;
                }

                outDeepPixel.reserve(nIntersections);

                // Build an output DeepSegment for each combined intersection:
                for (size_t i=0; i < nIntersections; ++i)
                {
                    const DeepIntersection& I = deepAccumIntersectionList[i];
                    Dcx::DeepSegment ds;
                    ds.Zf    = float(I.tmin);
                    ds.Zb    = float(I.tmax);
                    ds.index = -1; // gets assigned when appended to DeepPixel below
                    ds.metadata.spmask = I.spmask;
                    ds.metadata.flags  = Dcx::DEEP_LINEAR_INTERP_SAMPLE; // always hard surfaces for this example

                    // Append to DeepPixel and retrieve assigned index:
                    const size_t dsIndex = outDeepPixel.append(ds);

                    // Copy shaded color to DeepSegment's pixel:
                    Dcx::Pixelf& dp = outDeepPixel.getSegmentPixel(dsIndex);
                    int c = 0;
                    foreach_channel(z, Dcx::Mask_RGBA)
                        dp[z] = I.color[c++] / float(I.count);
                }
                //outDeepPixel.printInfo(std::cout, "outDeepPixel=");

                outDeepTile.setDeepPixel(outX, outY, outDeepPixel);


            } // outX loop

            // Write deep scanline so we can free tile line memory:
            outDeepTile.writeScanline(outY, true/*flush-line*/);

        } // outY loop

        // If all lines are flushed on write the tile should be using zero bytes
        // by end:
        if (verbose)
            std::cout << "  out_tile bytes=" << outDeepTile.bytesUsed() << std::endl;

    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        exitStatus = 1;
    }

    return exitStatus;
}
