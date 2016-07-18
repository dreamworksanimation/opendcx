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
/// @file DcxDeepImageTile.h

#ifndef INCLUDED_DCX_DEEPIMAGETILE_H
#define INCLUDED_DCX_DEEPIMAGETILE_H

//-----------------------------------------------------------------------------
//
//  class  DeepImageInputTile
//  class  DeepImageOutputTile
//
//-----------------------------------------------------------------------------

#include "DcxDeepTile.h"

#ifdef __ICC
// disable icc remark #1572: 'floating-point equality and inequality comparisons are unreliable'
//   this is coming from OpenEXR/half.h...
#   pragma warning(disable:2557)
#endif
#include <OpenEXR/ImfDeepImage.h>
#include <OpenEXR/ImfDeepImageLevel.h>
#include <OpenEXR/ImfDeepScanLineOutputFile.h>

OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

//-----------------------------------------------------------------------------
//
// class DeepImageInputTile
//
//      Adapter class for an input DeepImage tile.
//
//      Contains an array of TypedDeepImageChannel pointers, one for
//      each ChannelIdx entry.  Unassigned channels are left NULL.
//
//      TODO: support multiple DeepImages/Headers so that multiple Parts
//      can be combined into a single DeepPixel.
//
//-----------------------------------------------------------------------------

class DCX_EXPORT DeepImageInputTile : public DeepTile
{
  public:

    //
    // Must provide a ChannelContext at a minimum.
    //

    DeepImageInputTile (ChannelContext& channel_ctx,
                        bool tileYup=true);

    //
    // Constructs from a DeepImage (level 0) - assumes the
    // displayWindow == dataWindow.
    //

    DeepImageInputTile (const Imf::DeepImage& image,
                        ChannelContext& channel_ctx,
                        bool tileYup=true);

    //
    // Constructs from a DeepImage (level 0) and copies the displayWindow from
    // the separate Header.
    //
    // (TODO: this is awkward - DeepImage class should contain displayWindow!)
    //

    DeepImageInputTile (const Imf::Header& header,
                        const Imf::DeepImage& image,
                        ChannelContext& channel_ctx,
                        bool tileYup=true);


    //
    // Call this if the source DeepImage TypedDeepImageChannel have changed.
    //

    bool updateChannelPtrs ();


    //
    // Returns the number of deep samples at pixel x,y.
    //

    /*virtual*/ size_t getNumSamplesAt (int x, int y) const;


    //
    // Reads deep samples from a pixel-space location (x, y) into a deep pixel.
    // If xy is out of bounds the deep pixel is left empty and false is returned.
    //

    /*virtual*/ bool getDeepPixel (int x,
                                   int y,
                                   Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   ) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                        , bool debug=false
#endif
                                        ) const;


  protected:

    //
    // Copy constructor for subclasses
    //
    DeepImageInputTile (const DeepTile&);

    bool copyFromLevel (const Imf::DeepImage& image, int level);

    float getChannelSampleValueAt (int x, int y, size_t sample, const Imf::DeepImageChannel*) const;


  protected:

    const Imf::DeepImageLevel*                  m_image_level;          // The image level
    std::vector<const Imf::DeepImageChannel*>   m_chan_ptrs;            // Per-ChannelIdx channel data ptrs

};



//-----------------------------------------------------------------------------
//
// class DeepImageOutputTile
//
//      Adapter class for an output DeepImage tile.
//
//
//      TODO: support multiple DeepImages/Headers so that multiple Parts
//      can be combined into a single DeepPixel.
//
//-----------------------------------------------------------------------------


class DCX_EXPORT DeepImageOutputTile : public DeepTile
{
  public:

    typedef std::vector<half>     HalfVec;
    typedef std::vector<float>    FloatVec;
    typedef std::vector<uint32_t> UintVec;
    typedef std::vector<void*>    PtrVec;

    struct DeepLine
    {
        ChannelSet            channels;             // Channels which are in packed array
        std::vector<FloatVec> channel_arrays;       // Packed channel data for entire line
        std::vector<uint32_t> samples_per_pixel;    // Per-pixel sample count

        DeepLine (uint32_t width, const ChannelSet& _channels);

        uint32_t floatOffset (uint32_t xoffset) const;   // Get offset into channel_arrays for line x-offset

        void get (int xoffset,
                  Dcx::DeepPixel& deep_pixel) const;
        void getMetadata(int xoffset,
                         int sample,
                         Dcx::DeepMetadata& metadata) const;
        void set (int xoffset,
                  const Dcx::DeepPixel& deep_pixel);
        void clear (int xoffset);
    };


  public:

    //
    // Sets resolution and channel info.
    //

    DeepImageOutputTile (const IMATH_NAMESPACE::Box2i& display_window,
                         const IMATH_NAMESPACE::Box2i& data_window,
                         bool sourceWindowsYup,
                         const ChannelAliasSet& channels,
                         ChannelContext& channel_ctx,
                         bool tileYup=true);

    //
    // Copy resolution and channel info from another DeepTile.
    // No pixel data is copied.
    //

    DeepImageOutputTile (const DeepTile& tile);
    ~DeepImageOutputTile ();


    size_t      bytesUsed () const;


    DeepLine*   getLine (int y) const;


    //
    // Returns the number of deep samples at pixel x,y.
    //

    /*virtual*/ size_t getNumSamplesAt (int x, int y) const;


    //
    // Reads deep samples from a pixel-space location (x, y) into a deep pixel.
    // If xy is out of bounds the deep pixel is left empty and false is returned.
    // If write access is sequential 
    //

    /*virtual*/ bool getDeepPixel (int x,
                                   int y,
                                   Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   ) const;

    /*virtual*/ bool getSampleMetadata (int x,
                                        int y,
                                        size_t sample,
                                        Dcx::DeepMetadata& metadata
#ifdef DCX_DEBUG_DEEPTILE
                                        , bool debug=false
#endif
                                        ) const;


    //
    // Writes a DeepPixel to a pixel-space location (x, y) in the deep channels.
    // If xy is out of bounds false is returned.
    //

    /*virtual*/ bool setDeepPixel (int x,
                                   int y,
                                   const Dcx::DeepPixel& pixel
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   );


    //
    // Writes an empty DeepPixel (0 samples) at a pixel-space location (x, y).
    // If xy is out of bounds false is returned.
    //

    /*virtual*/ bool clearDeepPixel (int x,
                                     int y
#ifdef DCX_DEBUG_DEEPTILE
                                     , bool debug=false
#endif
                                     );

    //
    // Create an output deep file linked to this tile - destructive!
    // Will allocate a new Imf::DeepScanLineOutputFile and assign its
    // channels, destroying any current file.
    //

    virtual void    setOutputFile (const char* filename,
                                   Imf::LineOrder line_order=Imf::INCREASING_Y);


    //
    // Write a DeepLine to the output file.  Must be called in the line_order
    // specified in setOutputFile otherwise image will be upside-down.  ex. if
    // write line order is 0-100 use INCREASING_Y.  RANDOM_Y writing is only
    // supported for tiled images.
    // If flush_line is true the DeepLine memory is freed.
    //

    virtual void    writeScanline (int y,
                                   bool flush_line=true
#ifdef DCX_DEBUG_DEEPTILE
                                   , bool debug=false
#endif
                                   );


    //
    // Write entire tile to output file.
    // If flush_tile is true all DeepLine memory is freed.
    //

    virtual void    writeTile (bool flush_tile=true);


  protected:

    void        deleteDeepLines ();
    void        resizeDataWindow (const IMATH_NAMESPACE::Box2i& data_window);
    DeepLine*   createDeepLine (int y);


    std::vector<DeepLine*>          m_deep_lines;       // Channel data storage
    std::string                     m_filename;
    Imf::DeepScanLineOutputFile*    m_file;             // Output file, if assigned

};



//-----------------
// Inline Functions
//-----------------

inline
DeepImageInputTile::DeepImageInputTile (const DeepTile& b) : DeepTile(b) {}
inline
float DeepImageInputTile::getChannelSampleValueAt (int x, int y, size_t sample, const Imf::DeepImageChannel* c) const {
#ifdef DCX_DEBUG_DEEPTILE
    assert(c);
#endif
    if (m_tile_yUp)
        y = m_display_window.max.y - y;
    switch (c->pixelType())
    {
        case Imf::HALF:
            return float((*static_cast<const Imf::TypedDeepImageChannel<half>* >(c))(x, y)[sample]);
        case Imf::FLOAT:
            return (*static_cast<const Imf::TypedDeepImageChannel<float>* >(c))(x, y)[sample];
        case Imf::UINT:
            return float((*static_cast<const Imf::TypedDeepImageChannel<unsigned int>* >(c))(x, y)[sample]);
        default:
            assert(false);
    }
    return 0.0f;
}
//-----------------
inline
DeepImageOutputTile::DeepLine* DeepImageOutputTile::getLine(int y) const {
    return (y < m_data_window.min.y || y > m_data_window.max.y)?0:m_deep_lines[y - m_data_window.min.y]; }
//-----------------
inline
uint32_t
DeepImageOutputTile::DeepLine::floatOffset (uint32_t xoffset) const
{
#ifdef DCX_DEBUG_DEEPTILE
    assert(xoffset < samples_per_pixel.size());
#endif
    uint32_t offset = 0;
    const uint32_t* p = samples_per_pixel.data();
    for (uint32_t i=0; i < xoffset; ++i)
        offset += *p++;
    return offset;
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_DEEPIMAGETILE_H
