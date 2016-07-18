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
/// @file DcxChannelSet.h

#ifndef INCLUDED_DCX_CHANNELSET_H
#define INCLUDED_DCX_CHANNELSET_H

//-----------------------------------------------------------------------------
//
//  typedef  ChannelIdx
//  typedef  ChannelIdxSet
//  class    ChannelSet
//
//-----------------------------------------------------------------------------

#include "DcxAPI.h"

#include <OpenEXR/ImfPixelType.h>

#include <vector>
#include <set>
#include <iostream>

//#define DCX_DEBUG_CHANNEL_CREATION 1


OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER

//---------------------------------------------------------------------------

//
// Global channel-index type
//
typedef uint32_t ChannelIdx;

//
// Reserved value for a non-existant channel
//
static const ChannelIdx  Chan_Invalid = 0;

//
// Reserved value for the maximum channel index allowed.
// (TODO: remove this in favor of packed arrays...?)
//
static const ChannelIdx  Chan_Max = 1023;

//
// Reserved value. If this is the only ChannelIdx in a ChannelSet it
// indicates a set with all channels enabled (i.e. Mask_All)
// (TODO: ignored by most ChannelSet operations atm!)
//
static const ChannelIdx  Chan_All = 0xffffffff;


typedef std::set<ChannelIdx> ChannelIdxSet;


class ChannelContext;

//---------------------------------------------------------------------------
//
//  class ChannelSet
//      A std::set wrapper class that acts similar to DD::Image::ChannelSet.
//
//      The set contains ChannelIdx indices.  Use the foreach_channel()
//      macro to iterate through the set (similar to Nuke's
//      DD::Image foreach() macro.)
//
//---------------------------------------------------------------------------

class DCX_EXPORT ChannelSet
{
  public:

    //---------------------------------------------------------
    // Iterator-style access to channels in set
    //---------------------------------------------------------

    class iterator
    {
        friend class ChannelSet;
      public:
        iterator ();
        iterator (const ChannelIdxSet::iterator&);

        ChannelIdx  channel () const;
        ChannelIdx  operator *  ();

        bool operator != (const iterator&) const;
        bool operator != (ChannelIdx) const;
        bool operator == (const iterator&) const;
        bool operator == (ChannelIdx) const;

        friend std::ostream& operator << (std::ostream&,
                                          const iterator&);

      protected:
        ChannelIdxSet::iterator m_it;
    };

    typedef iterator const_iterator;


  public:

    //-----------------------------------------
    // Default constructor creates an empty set
    //-----------------------------------------

    ChannelSet ();

    //-----------------
    // Copy constructor
    //-----------------

    ChannelSet (const ChannelSet&);

    //-----------------------------------------------
    // Constructor to initialize from a ChannelIdxSet
    //-----------------------------------------------

    explicit ChannelSet (const ChannelIdxSet&);

    //-----------------------------------------------------
    // Constructor to initialize from a set of ChannelIdx's
    // (TODO: change to a C++11 initializer list...!)
    //-----------------------------------------------------
    ChannelSet (ChannelIdx a,
                ChannelIdx b=Chan_Invalid, ChannelIdx c=Chan_Invalid,
                ChannelIdx d=Chan_Invalid, ChannelIdx e=Chan_Invalid);


    //---------------
    // Copy operators
    //---------------

    ChannelSet& operator = (const ChannelSet&);
    ChannelSet& operator = (ChannelIdx);


    //----------------------------------
    // Remove all channels from the mask
    //----------------------------------

    void    clear ();


    //-------------------------------
    // Number of channels in the mask
    //-------------------------------

    size_t  size () const;


    //-------------------------------
    // Is mask empty?
    //-------------------------------

    bool    empty () const;


    //-----------------------------------------
    // Mask applies to all assigned ChannelIdxs
    //-----------------------------------------

    bool    all () const;


    //--------------------------------------------------------------
    // Read/write access to the wrapped std::set - use with caution!
    //--------------------------------------------------------------
    ChannelIdxSet&  mask ();


    //----------------------------
    // ChannelSet::iterator access
    //----------------------------

    iterator    first () const;
    iterator    last () const;
    iterator    prev (iterator it) const;
    iterator    next (iterator it) const;


    //--------------------------------------------------
    // Return true if the mask includes the ChannelIdx's
    //--------------------------------------------------

    bool    contains (const ChannelSet&) const;
    bool    contains (const ChannelIdxSet&) const;
    bool    contains (ChannelIdx) const;


    //--------------------------------------------------------------------
    // Add a ChannelIdx or ChannelSet to the mask.
    // There will only be one instance of each ChannelIdx value in the set
    //--------------------------------------------------------------------

    void    insert (const ChannelSet&);
    void    insert (const ChannelIdxSet&);
    void    insert (ChannelIdx);

    void    operator += (const ChannelSet&);
    void    operator += (const ChannelIdxSet&);
    void    operator += (ChannelIdx);


    //-----------------------------
    // Bitwise operators on the set
    //-----------------------------
    ChannelSet operator | (const ChannelSet&);
    ChannelSet operator & (const ChannelSet&);


    //------------------------------------------------
    // Remove a ChannelIdx or ChannelSet from the mask
    //------------------------------------------------
    void    erase (const ChannelSet&);
    void    erase (const ChannelIdxSet&);
    void    erase (ChannelIdx);
    void    operator -= (const ChannelSet&);
    void    operator -= (const ChannelIdxSet&);
    void    operator -= (ChannelIdx);


    //---------------------------------------------------
    // Intersect a ChannelIdx or ChannelSet with the mask
    //---------------------------------------------------
    void    intersect (const ChannelSet&);
    void    intersect (const ChannelIdxSet&);
    void    intersect (ChannelIdx);
    void    operator &= (const ChannelSet&);
    void    operator &= (const ChannelIdxSet&);
    void    operator &= (ChannelIdx);


    //--------------------------------------------------------------------
    // Print info about the set to an output stream
    //--------------------------------------------------------------------
    void print (const char* prefix,
                std::ostream&,
                const ChannelContext&) const;


  protected:

    ChannelIdxSet   m_mask;         // Unique set of ChannelIdx's

    static ChannelIdxSet m_npos;    // An iterator to this always returns 0 (Chan_Invalid)
};



//
//  Convenience macro for iterating through a ChannelSet.
// 
//      This is similar to Nuke's DD::Image::ChannelSet foreach() macro.
//      (must have a different name to avoid clashes when building Nuke plugins)
// 
//      Note that since an iterator is being incremented you must dereference it to get
//      the ChannelIdx value.
//      ex.
//          ChannelSet my_channels(Mask_RGBA);
//          Pixel<float> my_pixel(my_channels);
//          my_pixel.erase();
//          foreach_channel(z, my_channels)
//          {
//              my_pixel[*z] += 1.0f;
//          }
//

#undef  foreach_channel
#define foreach_channel(CHAN, CHANNELS) \
    for (OPENDCX_INTERNAL_NAMESPACE::ChannelSet::iterator CHAN=CHANNELS.first(); \
            *CHAN != OPENDCX_INTERNAL_NAMESPACE::Chan_Invalid; CHAN = CHANNELS.next(CHAN))


//
// ChannelSet bitwise operators
//

ChannelSet operator | (const ChannelSet&,
                       const ChannelSet&);
ChannelSet operator & (const ChannelSet&,
                       const ChannelSet&);




//-----------------
// Inline Functions
//-----------------

inline ChannelSet::ChannelSet () {}
inline ChannelSet::iterator::iterator () : m_it() {}
inline ChannelSet::iterator::iterator (const ChannelIdxSet::iterator& it) : m_it(it) {}
inline ChannelIdx ChannelSet::iterator::channel () const { return *m_it; }
inline ChannelIdx ChannelSet::iterator::operator * () { return this->channel(); }
inline bool ChannelSet::iterator::operator != (const iterator& b) const { return (m_it != b.m_it); }
inline bool ChannelSet::iterator::operator != (ChannelIdx channel) const { return (*m_it != channel); }
inline bool ChannelSet::iterator::operator == (const iterator& b) const { return (m_it == b.m_it); }
inline bool ChannelSet::iterator::operator == (ChannelIdx channel) const { return (*m_it == channel); }
//--------------------------------------------------------
inline void ChannelSet::insert (ChannelIdx channel) { if (channel > Chan_Invalid) m_mask.insert(channel); }
inline ChannelSet::ChannelSet (const ChannelSet& b) : m_mask(b.m_mask) {}
inline ChannelSet::ChannelSet (const ChannelIdxSet& mask) : m_mask(mask) {}
inline ChannelSet::ChannelSet (ChannelIdx a, ChannelIdx b, ChannelIdx c, ChannelIdx d, ChannelIdx e)
{
    this->insert(a); this->insert(b); this->insert(c); this->insert(d); this->insert(e);
}
//
inline ChannelSet& ChannelSet::operator = (const ChannelSet& b) { m_mask = b.m_mask; return *this; }
inline ChannelSet& ChannelSet::operator = (ChannelIdx channel) { m_mask.clear(); this->insert(channel); return *this; }
//
inline void   ChannelSet::clear () { m_mask.clear(); }
inline size_t ChannelSet::size () const { return m_mask.size(); }
inline bool   ChannelSet::empty () const { return (m_mask.size() == 0); }
inline bool   ChannelSet::all () const { return (m_mask.size() == 1 && *m_mask.begin() == Chan_All); }
inline ChannelIdxSet& ChannelSet::mask () { return m_mask; }
//
inline ChannelSet::iterator ChannelSet::first () const
{
    if (m_mask.size() == 0)
        return ChannelSet::iterator(m_npos.end());
    return ChannelSet::iterator(m_mask.begin());
}
inline ChannelSet::iterator ChannelSet::prev (iterator it) const
{
    if (it.m_it == m_mask.begin())
        return ChannelSet::iterator(m_mask.begin());
    else if (it.m_it == m_npos.end())
        return it;
    return ChannelSet::iterator(--it.m_it);
}
inline ChannelSet::iterator ChannelSet::next (iterator it) const
{
    if (it.m_it == m_npos.end() || it.m_it == m_mask.end())
        return ChannelSet::iterator(m_npos.end());
    if (++it.m_it == m_mask.end())
        return ChannelSet::iterator(m_npos.end());
    return ChannelSet::iterator(it.m_it);
}
inline ChannelSet::iterator ChannelSet::last () const
{
    if (m_mask.size() == 0)
        return ChannelSet::iterator(m_npos.end());
    return this->prev(ChannelSet::iterator(m_mask.end()));
}
//
inline bool   ChannelSet::contains (ChannelIdx channel) const { return (m_mask.find(channel) != m_mask.end()); }
inline bool   ChannelSet::contains (const ChannelIdxSet& b) const {
    for (ChannelIdxSet::const_iterator z=b.begin(); z != b.end(); ++z)
        if (m_mask.find(*z) == m_mask.end())
            return false;
    return true;
}
inline bool   ChannelSet::contains (const ChannelSet& b) const { return this->contains(b.m_mask); }
//
inline void   ChannelSet::insert (const ChannelIdxSet& b)
{
    for (ChannelIdxSet::const_iterator z=b.begin(); z != b.end(); ++z)
        this->insert(*z);
}
inline void   ChannelSet::insert (const ChannelSet& b) { this->insert(b.m_mask); }
inline void   ChannelSet::operator += (const ChannelSet& b) { this->insert(b.m_mask); }
inline void   ChannelSet::operator += (const ChannelIdxSet& b) { this->insert(b); }
inline void   ChannelSet::operator += (ChannelIdx channel) { this->insert(channel); }
//
inline void   ChannelSet::erase (const ChannelIdxSet& b)
{
    for (ChannelIdxSet::const_iterator z=b.begin(); z != b.end(); ++z)
        m_mask.erase(*z);
}
inline void   ChannelSet::erase (const ChannelSet& b) { this->erase(b.m_mask); }
inline void   ChannelSet::erase (ChannelIdx channel) { m_mask.erase(channel); }
inline void   ChannelSet::operator -= (ChannelIdx channel) { m_mask.erase(channel); }
inline void   ChannelSet::operator -= (const ChannelIdxSet& b) { this->erase(b); }
inline void   ChannelSet::operator -= (const ChannelSet& b) { this->erase(b.m_mask); }
//
inline void   ChannelSet::intersect (const ChannelIdxSet& b)
{
    std::vector<ChannelIdx> erase_list;
    erase_list.reserve(m_mask.size());
    for (ChannelIdxSet::const_iterator z=m_mask.begin(); z != m_mask.end(); ++z)
        if (b.find(*z) == b.end())
            erase_list.push_back(*z);
    for (size_t i=0; i < erase_list.size(); ++i)
        m_mask.erase(erase_list[i]);
}
inline void   ChannelSet::intersect (const ChannelSet& b) { this->intersect(b.m_mask); }
inline void   ChannelSet::intersect (ChannelIdx chan)
{
    if (m_mask.find(chan) != m_mask.end())
    {
        m_mask.clear();
        m_mask.insert(chan);
    }
    else
        m_mask.clear();
}
inline void   ChannelSet::operator &= (const ChannelSet& b) { this->intersect(b.m_mask); }
inline void   ChannelSet::operator &= (const ChannelIdxSet& b) { this->intersect(b); }
inline void   ChannelSet::operator &= (ChannelIdx channel) { this->intersect(channel); }
//
inline ChannelSet ChannelSet::operator | (const ChannelSet& b) { this->insert(b); return *this; }
inline ChannelSet ChannelSet::operator & (const ChannelSet& b) { this->intersect(b); return *this; }
//--------------------------------------------------------
inline ChannelSet operator | (const ChannelSet& a, const ChannelSet& b)
{
    ChannelSet c(a);
    c.insert(b);
    return c;
}
inline ChannelSet operator & (const ChannelSet& a, const ChannelSet& b)
{
    ChannelSet c(a);
    c.intersect(b);
    return c;
}


OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT

#endif // INCLUDED_DCX_CHANNELSET_H
