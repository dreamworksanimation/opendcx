///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016 DreamWorks Animation LLC. 
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of DreamWorks Animation nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
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
/// @file version.h

#ifndef OPENDCX_VERSION_H
#define OPENDCX_VERSION_H

//==================================================================================
// Define our version symbols
// Keep major & minor in sync with OpenEXR / IlmBase!
// Patch will be used to increment the OpenDCX release number.
#define OPENDCX_VERSION        "2.02.02"
#define OPENDCX_VERSION_INT    20202
#define OPENDCX_VERSION_MAJOR  2
#define OPENDCX_VERSION_MINOR  2
#define OPENDCX_VERSION_PATCH  2

//==================================================================================
// Define namespace macros
#ifndef OPENDCX_NAMESPACE
#define OPENDCX_NAMESPACE Dcx
#endif

#ifndef OPENDCX_INTERNAL_NAMESPACE
#define OPENDCX_INTERNAL_NAMESPACE OPENDCX_NAMESPACE
#endif

//
// We need to be sure that we import the internal namespace into the public one.
// To do this, we use the small bit of code below which initially defines
// OPENDCX_INTERNAL_NAMESPACE (so it can be referenced) and then defines
// OPENDCX_NAMESPACE and pulls the internal symbols into the public
// namespace.
//

namespace OPENDCX_INTERNAL_NAMESPACE {}
namespace OPENDCX_NAMESPACE {
    using namespace OPENDCX_INTERNAL_NAMESPACE;
}

//
// There are identical pairs of HEADER/SOURCE ENTER/EXIT macros so that
// future extension to the namespace mechanism is possible without changing
// project source code.
//

#define OPENDCX_INTERNAL_NAMESPACE_HEADER_ENTER namespace OPENDCX_INTERNAL_NAMESPACE {
#define OPENDCX_INTERNAL_NAMESPACE_HEADER_EXIT }

#define OPENDCX_INTERNAL_NAMESPACE_SOURCE_ENTER namespace OPENDCX_INTERNAL_NAMESPACE {
#define OPENDCX_INTERNAL_NAMESPACE_SOURCE_EXIT }


#endif // OPENDCX_VERSION_H
