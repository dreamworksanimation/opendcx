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
/// @file microPolyAccumulator.h

#ifndef INCLUDED_MICROPOLYACCUMULATOR_H
#define INCLUDED_MICROPOLYACCUMULATOR_H

//-----------------------------------------------------------------------------
//
//  class   microPolyAccumulator
//  class   microPolyAccumulator::Quad
//  class   microPolyAccumulator::Fragment
//  struct  microPolyAccumulator::FragmentCluster
//
//-----------------------------------------------------------------------------

#include <OpenDCX/DcxSpMask.h>
#include <OpenDCX/DcxDeepPixel.h>

#include <vector>
#include <map>


//-----------------------------------------------------------------------------
//
// class microPolyAccumulator
//
//      The fragment-to-sample conversion class
//
//      Accumulates a series of surface fragments and combines them into a
//      series of deep samples.
//
//-----------------------------------------------------------------------------

class microPolyAccumulator
{
  public:

    //---------------------------------------------------------------------------
    //
    // class Quad
    //
    //      This represents a complete micropolygon primitive.  Micropolygons are
    //      usually about the size of one pixel, which results in them straddling
    //      3 or 4 pixels when rasterized with no motion blur.
    //      The full implementation of the Quad class should maintain pointers to
    //      vertices, a geometry index, and possibly material info etc.
    //
    //---------------------------------------------------------------------------

    class Quad
    {
      public:
        virtual ~Quad ();

        //
        // Calculate the min/max z from all the vertices, using the camera transform at a single dt (time delta)
        // Must be implemented.
        //

        virtual void getCameraSpaceMinMaxZ (double& minZ,
                                            double& maxZ) const=0;


        //
        // Return a unique geometry identifier index.
        // This allows merging of samples that come from the same geometry object.
        // Must be implemented.
        //

        virtual uint64_t geometryIdx () const=0;


        //
        // Adjacency determined by vertex sharing, or by vertices within epsilon of each other
        // and can optionally use other criteria such as material matching.
        // Must be implemented.
        //

        virtual bool isAdjacent (const Quad* otherQuad) const=0;
    };


    //------------------------------------------------------------------------------------
    //
    // class Fragment
    //
    //      This represents all sub-pixel samples for a given Quad intersecting the
    //      current pixel, and is built via stratified sampling across the sub-pixel grid.
    //      Temporal stratification means that (due to motion blur) the sub-pixel mask
    //      might not be contiguous, and that the range of min/max Z might be expanded
    //      far beyond the range of the corresponding Quad.
    //
    //      Even without motion blur, it is expected that up to 4 Fragments in adjacent
    //      pixels will reference the same Quad.  In the presence of motion blur there
    //      may be many more adjacent pixels with Fragments referencing the same Quad.
    //      This expansion of range results in significantly more overlaps between samples
    //      making the fragment Z range less desirable in determining which samples should
    //      be consolidated.
    //
    //------------------------------------------------------------------------------------

    class Fragment
    {
      public:
        Fragment (const Dcx::Pixelf& chans,
                  const Dcx::SpMask8& spmask);
        virtual ~Fragment ();


        //
        // The subpixel bitmask pattern
        //

        virtual Dcx::SpMask8 spMask () const=0;


        //
        // Return the shaded color of the fragment.
        // This also contains depth information and the per-component alphas (AR, AG, AB).
        //

        virtual const Dcx::Pixelf& color () const=0;


        //
        // Retrieve the geometry object
        //

        virtual const Quad* quad () const=0;


        //
        // Returns the min/max z of all the subpixels affected by this fragment
        //

        virtual void getCameraSpaceMinMaxZ (double& minZ,
                                            double& maxZ) const;
    };


    //------------------------------------------------------------------------
    //
    // class FragmentCluster
    //
    //      Fragment clustering - allows fragments to be joined into clusters.
    //      Fragments that are identified as sharing traits will be put into
    //      the same cluster.
    //      The cluster maintains its own min/max z value
    //
    //------------------------------------------------------------------------

    struct FragmentCluster
    {

        //
        // Combine with the fragments from another cluster
        //

        void    merge (const FragmentCluster& src);


        //
        // OR all the fragment masks together.
        //

        Dcx::SpMask8 generateCombinedMask ();


        //
        // Composite the fragments together
        //

        void    composite (Dcx::Pixelf& result,
                           Dcx::SpMask8& accumMask) const;


        //
        // Compare structs for sorting a vector of FragmentCluster
        //

        struct CompareQuadZ
        {
            // Compare two fragment clusters according to their primitive depth.
            bool operator() (const FragmentCluster& a,
                             const FragmentCluster& b) const;
        };
        struct CompareSurfaceZ
        {
            // Compare two fragment clusters according to their surface depth.
            bool operator() (const FragmentCluster& a,
                             const FragmentCluster& b) const;
        };


        std::vector<const Fragment*>    fragments;      // A list of the fragments that belong to the cluster
        uint64_t                        geometryIdx;    // Unique geometry identifier

        // the Z range occupied by the cluster
        double                          quadMinZ;       // Contiguous primitive Zf
        double                          quadMaxZ;       // Contiguous primitive Zb
        double                          surfMinZ;       // Distributed geometry Zf
        double                          surfMaxZ;       // Distributed geometry Zb

    };

    typedef std::vector<FragmentCluster>            FragmentClusterVec;
    typedef std::map<uint64_t, FragmentClusterVec>  FragmentClusterMap;


    //-----------------------------------------------------------------------------


  public:

    microPolyAccumulator ();
    virtual ~microPolyAccumulator ();


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
    //
    // In the case where a fragment does not match any existing cluster, a new cluster of
    // one fragment is created.
    //

    virtual void    accumulate (const Fragment* fragment);


    //
    // Consolidate any surface clusters that have been accumulated so far
    //
    // Maximum depth complexity:
    //   0 means infinite - i.e. no gap removal!
    //   1 means a single cluster and no gaps
    //   3 is the default and means 3 clusters (i.e. 2 gaps)
    //

    virtual void    consolidate (FragmentClusterVec& sortedClusters,
                                 int maxDepthComplexity=3);


    //
    // Convert surface clusters to DeepSegments and add to DeepPixel
    //

    virtual void    output (const FragmentClusterVec& clusters,
                            Dcx::DeepPixel& out_deep_pixel);


    //
    // Clear any surface clusters
    //

    virtual void    flush ();


  protected:

    FragmentClusterMap  mClusterMap;

};



//-----------------
// Inline Functions
//-----------------

/*virtual*/
inline
microPolyAccumulator::Quad::~Quad () {}
//--------------------------------------------------------
inline
void
microPolyAccumulator::FragmentCluster::merge (const FragmentCluster& src)
{
    fragments.insert(fragments.end(), src.fragments.begin(), src.fragments.end());
    quadMinZ = std::min(quadMinZ, src.quadMinZ);
    quadMaxZ = std::max(quadMaxZ, src.quadMaxZ);
    surfMinZ = std::min(surfMinZ, src.surfMinZ);
    surfMaxZ = std::max(surfMaxZ, src.surfMaxZ);
}
inline
Dcx::SpMask8
microPolyAccumulator::FragmentCluster::generateCombinedMask ()
{
    Dcx::SpMask8 resultMask = 0;
    for (size_t s=0; s < fragments.size(); ++s)
        resultMask |= fragments[s]->spMask();
    return resultMask;
}
inline
bool
microPolyAccumulator::FragmentCluster::CompareQuadZ::operator() (const FragmentCluster& a,
                                                                 const FragmentCluster& b) const
{ return (a.quadMinZ < b.quadMinZ); }
inline
bool
microPolyAccumulator::FragmentCluster::CompareSurfaceZ::operator() (const FragmentCluster& a,
                                                                    const FragmentCluster& b) const
{ return (a.surfMinZ < b.surfMinZ); }


#endif // INCLUDED_MICROPOLYACCUMULATOR_H
