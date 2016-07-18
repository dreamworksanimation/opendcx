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
/// @file microPolyAccumulator.cpp

#include "microPolyAccumulator.h"

#include "OpenEXR/ImathVec.h"

#include <algorithm> // for std::sort in some compilers
#include <assert.h>


using namespace Dcx;

void
microPolyAccumulator::FragmentCluster::composite (Dcx::Pixelf& result,
                                                  SpMask8& accumMask) const
{
    IMATH_NAMESPACE::V3f subpxAlpha[SpMask8::numBits];
    memset(subpxAlpha, 0, sizeof(subpxAlpha));
    accumMask = SpMask8::allBitsOff;

    size_t spindex;

    // Build output channel set from of all fragment channels:
    ChannelSet comp_channels(Mask_RGBA);  // default to RGBA
    result.erase();

    for (size_t s=0; s < fragments.size(); ++s)
    {
        const Fragment* fragment = fragments[s];

        const Pixelf& fragColor = fragment->color();
        const SpMask8 fragMask  = fragment->spMask();

        result.channels += fragColor.channels;

        ChannelSet fragChannelsNoRgbNoAlphas(fragColor.channels);
        fragChannelsNoRgbNoAlphas -= Mask_RGB;
        fragChannelsNoRgbNoAlphas -= Mask_Alphas;

        // perform subpixel compositing for this fragment
#if 1
        SpMask8 sp(1ull);
        for (spindex=0; spindex < SpMask8::numBits; ++spindex, ++sp)
        {
            IMATH_NAMESPACE::V3f& alphas = subpxAlpha[spindex];
            if (fragMask & sp)
            {
                // Combined alpha is average of accumulated opacities:
                const float invAvgA = 1.0f - (alphas[0] + alphas[1] + alphas[2]) / 3.0f;

                // Composite AOVs with combined alpha (skipping rgb & alphas):
                foreach_channel(z, fragChannelsNoRgbNoAlphas)
                    result[*z] += fragColor[*z] * invAvgA;

                // Composite RGB with separate alphas:
                result[Chan_R] += fragColor[Chan_R] * (1.0f - alphas[0]);
                result[Chan_G] += fragColor[Chan_G] * (1.0f - alphas[1]);
                result[Chan_B] += fragColor[Chan_B] * (1.0f - alphas[2]);

                // Composite alphas:
                if (fragColor.channels.contains(Mask_RGBAlphas))
                {
                    alphas[0] += (1.0f - alphas[0]) * fragColor[Chan_AR];
                    alphas[1] += (1.0f - alphas[1]) * fragColor[Chan_AG];
                    alphas[2] += (1.0f - alphas[2]) * fragColor[Chan_AB];
                }
                else
                {
                    alphas[0] += (1.0f - alphas[0]) * fragColor[Chan_A];
                    alphas[1] += (1.0f - alphas[1]) * fragColor[Chan_A];
                    alphas[2] += (1.0f - alphas[2]) * fragColor[Chan_A];
                }
            }
        }
#else
        for (size_t sy=0, spindex=0; sy < SpMask8::width; ++sy)
        {
            for (size_t sx=0; sx < SpMask8::width; ++sx, ++spindex)
            {
                if (fragMask.isSubpixelOn(sx, sy))
                {
                    const float AR = subpxAlpha[spindex][0];
                    const float AG = subpxAlpha[spindex][1];
                    const float AB = subpxAlpha[spindex][2];
                    const float A = (AR + AG + AB) / 3.0f;
                    result += fragColor * (1.0f - A);
                    // Composite alphas:
                    subpxAlpha[spindex][0] += (1.0f - AR) * fragAlpha[Chan_R];
                    subpxAlpha[spindex][1] += (1.0f - AG) * fragAlpha[Chan_G];
                    subpxAlpha[spindex][2] += (1.0f - AB) * fragAlpha[Chan_B];
                }
            }
        }
#endif
        accumMask |= fragMask;
    }
    assert(accumMask != SpMask8::allBitsOff); // shouldn't happen...

    // Add up final alphas from all subpixels:
    IMATH_NAMESPACE::V3f outAlpha(0, 0, 0);
    for (spindex=0; spindex < SpMask8::numBits; ++spindex)
        outAlpha += subpxAlpha[spindex];

    const float invNumSubPixels = 1.0f / (float)SpMask8::numBits;
    result   *= invNumSubPixels;
    outAlpha *= invNumSubPixels;

    // Replace output alphas with combined alphas from all subpixels:
    result[Dcx::Chan_AR] = outAlpha[0];
    result[Dcx::Chan_AG] = outAlpha[1];
    result[Dcx::Chan_AB] = outAlpha[2];
    result.channels += Mask_RGBAlphas;
    // Final alpha is average of final opacities:
    result[Dcx::Chan_A] = (outAlpha[0] + outAlpha[1] + outAlpha[2]) / 3.0f;
    result.channels += Mask_A;
}


//-----------------------------------------------------------------------------


microPolyAccumulator::microPolyAccumulator ()
{
    //
}

/*virtual*/
microPolyAccumulator::~microPolyAccumulator ()
{
    //
}



//
// Surface fragments will be queued up to determine if contiguous.
//
// Fragments will be organized into clusters.  Each cluster contains a list of
// fragments, and a min/max z value.  Each incoming fragment is tested against
// existing clusters to determine if there is a match between any of the vertices.
// When a fragment is found to match one or more clusters, the fragment and all
// matching clusters are consolidated into a single cluster.  The process is repeated
// for every incoming surface fragment until all have been processed.  During this
// process the min/max z value of each cluster is updated based on the contained fragments.
// In the case where a fragment does not match any existing cluster, a new cluster of
// one fragment is created.
//
// Both geometry and surface Z have their uses.  It does however make the sample code harder to follow.
// We could potentially modify the code to use a single Z range and leave it up to the user to
// refine the algorithm based on their individual needs.  Certainly the distinction between surface
// and geometry makes little sense in a ray tracer.
//

/*virtual*/
void
microPolyAccumulator::accumulate (const Fragment* fragment)
{
    // extract some basic information from the surface fragment
    const Quad* thisQuad = fragment->quad();
    double  quadMinZ;
    double  quadMaxZ;
    double  surfMinZ;
    double  surfMaxZ;

    thisQuad->getCameraSpaceMinMaxZ(quadMinZ, quadMaxZ);
    fragment->getCameraSpaceMinMaxZ(surfMinZ, surfMaxZ);

    // Clusters are thrown into a std::map where geometry_idx is the key
    FragmentClusterVec& clusterVec = mClusterMap[thisQuad->geometryIdx()];


    int     clusterCount = (int)clusterVec.size();
    bool    matchesAnyCluster = false;
    bool    firstMatch = true;
    int     firstMatchClusterIndex = -1;

    for (int c=0; c < clusterCount; ++c)
    {

        bool matchesCurCluster = false;

        const size_t fragmentCount = clusterVec[c].fragments.size();
        for (size_t f=0; f < fragmentCount; ++f)
        {
            const Fragment* otherSurface = clusterVec[c].fragments[f];
            const Quad*     otherGeo     = otherSurface->quad();
            if (thisQuad->isAdjacent(otherGeo))
            {
                // these geos form a contiguous surface
                matchesCurCluster = true;
                matchesAnyCluster = true;
                break;
            }

        }

        // did we find a match?
        if (matchesCurCluster)
        {
            // it is a match, so we augment the existing cluster by adding
            // the fragment and updating the minz/maxz range

            if (firstMatch)
            {
                clusterVec[c].fragments.push_back(fragment);
                clusterVec[c].surfMinZ = std::min(clusterVec[c].surfMinZ, surfMinZ);
                clusterVec[c].surfMaxZ = std::max(clusterVec[c].surfMaxZ, surfMaxZ);
                clusterVec[c].quadMinZ = std::min(clusterVec[c].quadMinZ, quadMinZ);
                clusterVec[c].quadMaxZ = std::max(clusterVec[c].quadMaxZ, quadMaxZ);

                firstMatchClusterIndex = c;
                firstMatch = false;
            }
            else
            {
                // If this is not the first match, it means that the new fragment is the missing piece that
                // joins two or more existing clusters together.  Go ahead and merge the clusters.
                clusterVec[firstMatchClusterIndex].merge(clusterVec[c]);

                // remove current match and decrement index
                clusterVec.erase(clusterVec.begin() + c);
                c--;
                clusterCount--;
            }

            // now look for other clusters...
        }
    }
    if (!matchesAnyCluster)
    {
        // not a match, so we add a new cluster
        FragmentCluster newCluster;
        newCluster.fragments.push_back(fragment);
        newCluster.geometryIdx = thisQuad->geometryIdx();
        newCluster.surfMinZ    = surfMinZ;
        newCluster.surfMaxZ    = surfMaxZ;
        newCluster.quadMinZ    = quadMinZ;
        newCluster.quadMaxZ    = quadMaxZ;
        clusterVec.push_back(newCluster);
    }
}


//
// Consolidate any surface clusters that have been accumulated so far
//
// Maximum depth complexity:
//   0 means infinite - i.e. no gap removal!
//   1 means a single cluster and no gaps
//   3 is the default and means 3 clusters (i.e. 2 gaps)
//
// This is not a perfect solution, but improves the worst case (without doing much in better cases).
//
// Let us assume that if fragments from 1 surface are not all contiguous, that there should only be
// a finite number of non-contiguous spans.  A case where non-contiguous spans are expected would be
// for example on silhouette edges of concave geometry where you would see more fragments from the
// same geometry further in the distance.  A case where non-contiguous spans are not expected (and
// are undesirable) would be where coarse sampling of motion micropolygons results in fragments
// built from semi-adjacent deep geometry.  The contiguity test is only capable of detecting geometry
// that are adjacent, so semi-adjacent geometry will form discrete single-geometry clusters.
// Fortunately these clusters will likely have very small gaps between them along the Z axis.
// 
// If we assume that there may be a large number of very small gaps (due to motion sampling) and
// a small number of very large gaps (due to concave geometry) it should be possible to construct
// an algorithm that consolidates smaller gaps and preserves larger gaps.  Ideally the algorithm
// should be scale agnostic, and not rely on too many hard coded numbers (particularly numbers that
// relate to world scale).  Also it would be preferable to not require the user to supply hand-tuned
// values in order to produce acceptable results.  In order to achieve a balance here the proposed
// algorithm will start by removing the smallest gap and continue to remove gaps of increasing size
// until there are no ore than N gaps (where N is a small number, possibly 1 or 2).  This process
// is repeated for each geometry_idx (where gaps are evaluated only between clusters that share the
// same geometry_idx).
//
// The advantage of this process is that it puts a cap on the maximum number of samples that can be
// contributed from any given geometry_idx.  This means that the explosion of sample counts generated
// from irresponsible amounts of motion blur will quickly hit the predetermined limit and then remain
// constant.  The main disadvantage to this approach relates to the phenomenon that micropolygons
// are occasionally dropped due to absence of subpixel coverage.  This causes the contiguity test
// to fail and our algorithm will always allow N gaps to remain unconsolidated.
// The fallout from this is small nonetheless; subpixel dropout is fairly infrequent, and is more
// common along silhouette edges where there is the potential for additional consolidation through
// other means.
//

/*virtual*/
void
microPolyAccumulator::consolidate (FragmentClusterVec& sortedClusters,
                                   int maxDepthComplexity)
{
    sortedClusters.clear();
    if (mClusterMap.size() == 0)
        return;

    // Gap reduction per geometry_idx

    FragmentClusterMap::iterator it;

    if (maxDepthComplexity > 0)
    {
        for (it = mClusterMap.begin(); it != mClusterMap.end(); ++it)
        {
            // process one geometry_idx at a time
            FragmentClusterVec& clusterVec = it->second;

            // Sort clusters based on their surfMinZ
            FragmentCluster::CompareSurfaceZ cmpS;
            std::sort(clusterVec.begin(), clusterVec.end(), cmpS);

            while ((int)clusterVec.size() > maxDepthComplexity)
            {
                // there are too many gaps - find the smallest and remove it
                // note: this also removes overlaps (within the same geometry_idx)
                float   bestDiff = std::numeric_limits<float>::max();
                int     bestIndex = -1;
                for (int c=0; c < (int)clusterVec.size()-1; ++c)
                {
                    float delta = clusterVec[c+1].surfMinZ - clusterVec[c].surfMaxZ;
                    if (delta < bestDiff)
                    {
                        bestDiff = delta;
                        bestIndex = c;
                    }
                }
                // we have the best index to consolidate...
                clusterVec[bestIndex].merge(clusterVec[bestIndex+1]);

                // remove current match
                clusterVec.erase(clusterVec.begin() + bestIndex + 1);
            }
        }
    }

    // Consolidate overlapping clusters (across multiple geometry_idxs)
    //
    // Now do this by copying all clusters to a single list, sorting by min-z
    // and finding overlaps with neighbors
    for (it = mClusterMap.begin(); it != mClusterMap.end(); ++it)
    {
        FragmentClusterVec& clusterVec = it->second;
        for (int c=0; c < (int)clusterVec.size(); ++c)
            sortedClusters.push_back(clusterVec[c]);
    }

    // Consolidate clusters that overlap based on the quadZ range
    if (sortedClusters.size() > 1) {
        // Consolidate overlapping clusters (within matching geometry_idxs)
        // Sort clusters by min-z and find overlaps with neighbors where subpixels don't overlap

        // only consolidate when clusters of overlapping z range *don't* overlap subpixels
        FragmentCluster::CompareQuadZ cmpP;
        std::sort(sortedClusters.begin(), sortedClusters.end(), cmpP);

        SpMask8 m1, m2;
        for (int c=0; c < (int)sortedClusters.size()-1; ++c)
        {
            if (sortedClusters[c+1].geometryIdx != sortedClusters[c].geometryIdx)
                continue;
            // Same geometryIdx, are they overlapping?
            if (sortedClusters[c+1].quadMinZ <= sortedClusters[c].quadMaxZ)
            {
                // Check subpixels
                m1 = sortedClusters[c].generateCombinedMask();
                m2 = sortedClusters[c+1].generateCombinedMask();
                if ((m1 & m2) == SpMask8::allBitsOff)
                {
                    sortedClusters[c].merge(sortedClusters[c+1]);
                    sortedClusters.erase(sortedClusters.begin() + c + 1);
                    c--;
                }
            }
        }
        FragmentCluster::CompareSurfaceZ cmpS;
        std::sort(sortedClusters.begin(), sortedClusters.end(), cmpS);
    }

}


//
// Convert surface clusters to DeepSegments and add to DeepPixel:
//

/*virtual*/
void
microPolyAccumulator::output (const FragmentClusterVec& clusters,
                              DeepPixel& out_deep_pixel)
{
    const size_t nClusters = clusters.size();
    if (nClusters == 0)
        return;

    Dcx::DeepSegment ds;
    Dcx::Pixelf dp(out_deep_pixel.channels());
    dp.channels += Dcx::Mask_RGBAlphas;
    SpMask8 accumMask;

    for (size_t c=0; c < nClusters; ++c)
    {
        const FragmentCluster& fc = clusters[c];

        // Determine average rgb and coverage for all fragments within cluster:
        fc.composite(dp, accumMask);

        // Unpremultiply the rgb values by the coverage:
        if (accumMask != SpMask8::zeroCoverage)
            dp *= accumMask.toCoverage();

        ds.Zf              = fc.surfMinZ;
        ds.Zb              = fc.surfMaxZ;
        ds.index           = -1; // this gets updated when added to DeepPixel
        ds.metadata.spmask = accumMask;
        ds.metadata.flags  = DEEP_LINEAR_INTERP_SAMPLE; // always solid surfaces

        // Add the segment and pixel channel data to the output deep pixel:
        out_deep_pixel.append(ds, dp);
    }

}


//
// Clear any surface clusters
//

/*virtual*/
void
microPolyAccumulator::flush ()
{
    mClusterMap.clear();
}


//----------------------------------------------------------------------------
