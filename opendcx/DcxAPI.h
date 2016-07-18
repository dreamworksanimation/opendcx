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
/// @file DcxAPI.h

#ifndef INCLUDED_DCX_API_H
#define INCLUDED_DCX_API_H

/* Windows compatibility.
   When compiling the library DWA_DCX_EXPORT is defined with -D
   When compiling programs DWA_DCX_EXPORT is undefined
*/
#ifndef DCX_EXPORT
#   if defined(_WIN32)
#       if defined(DWA_OPENDCX_EXPORTS)
#           define DCX_EXPORT __declspec(dllexport)
#       else
#           define DCX_EXPORT __declspec(dllimport)
#       endif
#   else
#       define DCX_EXPORT
#   endif
#endif

#include <limits>  // for std::numeric_limits

#include "version.h"

//==================================================================================
// Define epsilon for floats & doubles - for convenience
#undef  EPSILONf
#define EPSILONf std::numeric_limits<float>::epsilon()//0.0001f
#undef  EPSILONd
#define EPSILONd std::numeric_limits<double>::epsilon()//0.000001

// Define infinity for floats & doubles - for convenience
#undef  INFINITYf
#define INFINITYf std::numeric_limits<float>::infinity()//1e+37
#undef  INFINITYd
#define INFINITYd std::numeric_limits<double>::infinity()//1e+37


#endif // INCLUDED_DCX_API_H
