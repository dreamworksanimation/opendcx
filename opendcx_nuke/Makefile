# Copyright (c) 2016 DreamWorks Animation LLC. 
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# *       Redistributions of source code must retain the above
#         copyright notice, this list of conditions and the following
#         disclaimer.
# *       Redistributions in binary form must reproduce the above
#         copyright notice, this list of conditions and the following
#         disclaimer in the documentation and/or other materials
#         provided with the distribution.
# *       Neither the name of DreamWorks Animation nor the names of its
#         contributors may be used to endorse or promote products
#         derived from this software without specific prior written
#         permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Makefile for OpenDCX Nuke plugins
#

#---------------------------------------------------------------------------
# DWA local build settings, remove before publishing:
CXX              := /rel/lang/gcc/gcc412/bin/g++412
LDFLAGS          := -Wl,-rpath,/rel/lang/gcc/gcc412/lib64 -L/rel/lang/gcc/gcc412/lib64
NUKE_LOCATION    := /rel/third_party/nuke/9.0v6_64
OPENEXR_INCL_DIR := /rel/folio/openexr/openexr-2.2.0.d.w.a.5.0-latest/include
OPENEXR_LIB_DIR  := /rel/folio/openexr/openexr-2.2.0.d.w.a.5.0-latest/lib
#---------------------------------------------------------------------------


# Targets:
#   all
#   install             install the plugins into NUKE_DESTDIR
#   clean               delete generated files from the local directory
#   depend              recompute source file header dependencies
#
# Options:
#   shared=no           link executables against static OpenDCX libraries
#                       (default: link against shared libraries)
#   debug=yes           build with debugging symbols and without optimization
#   verbose=yes         run commands in verbose mode


#
# The following variables must be defined, either here or on the command line
# (e.g., "make install DESTDIR=/usr/local"):
#
# Note that the OpenDCX library and the Nuke plugins must be built
# against compatible versions of the OpenEXR library.
#


# NUKE_LOCATION should be set to the directory where Nuke is installed.
ifeq (,$(NUKE_LOCATION))
$(warning Warning: $$(NUKE_LOCATION) is not set)
endif


# The directory into which to install libraries and header files
# (it is recommended that this match the DESTDIR in the OpenDCX
# library makefile)
DESTDIR := /tmp/OpenDCX

# The directory into which to install Nuke plugins, scripts, etc.
NUKE_DESTDIR := $(DESTDIR)/nuke

ifndef OPENDCX_INCL_DIR
    # The parent directory of the OpenDCX/ header directory
    OPENDCX_INCL_DIR := $(DESTDIR)/include
endif
ifndef OPENDCX_LIB_DIR
    # The directory containing libOpenDCX
    OPENDCX_LIB_DIR := $(DESTDIR)/lib
endif


# The version of OpenEXR should match the version of OpenDCX.
# i.e. OpenDCX 2.2 requires OpenEXR 2.2
ifndef OPENEXR_INCL_DIR
    # The parent directory of the OpenEXR header directory
    OPENEXR_INCL_DIR := /usr/include
endif
ifndef OPENEXR_LIB_DIR
    # The directory containing IlmImf & IlmImfUtil libs
    OPENEXR_LIB_DIR := /usr/lib64
endif

# static or shared lib linking
ifeq (no,$(strip $(shared)))
    OPENDCX_LIBS := $(OPENDCX_LIB_DIR)/libOpenDCX.a
    OPENEXR_LIBS := \
        $(OPENEXR_LIB_DIR)/libIlmImfUtil.a \
        $(OPENEXR_LIB_DIR)/libIlmImf.a \
        $(OPENEXR_LIB_DIR)/libIex.a \
        $(OPENEXR_LIB_DIR)/libImath.a \
        $(OPENEXR_LIB_DIR)/libHalf.a \
        $(OPENEXR_LIB_DIR)/libIlmThread.a
else
    OPENDCX_LIBS := -lOpenDCX
    OPENEXR_LIBS := \
        -lIlmImfUtil \
	-lIlmImf \
	-lIex \
	-lImath \
	-lHalf \
	-lIlmThread
endif #shared


#
# Ideally, users shouldn't need to change anything below this line.
#

SHELL = /bin/bash

# Turn off implicit rules for speed
.SUFFIXES:

ifneq (,$(INSTALL_DIR))
    $(warning Warning: $$(INSTALL_DIR) is no longer used; set $$(DESTDIR) instead.)
endif

# Determine the platform.
ifeq ("$(OS)","Windows_NT")
    WINDOWS_NT := 1
else
    UNAME_S := $(shell uname -s)
    ifeq ("$(UNAME_S)","Linux")
        LINUX := 1
    else
        ifeq ("$(UNAME_S)","Darwin")
            MBSD := 1
        endif
    endif
endif

NUKE_INCL_DIR := $(NUKE_LOCATION)/include

ifeq (yes,$(strip $(debug)))
    OPTIMIZE := -g
else
    OPTIMIZE := -O3 -DNDEBUG
endif

ifeq (yes,$(strip $(verbose)))
    QUIET :=
else
    QUIET := > /dev/null
endif

INCLDIRS := \
    -I$(OPENDCX_INCL_DIR) \
    -I$(OPENEXR_INCL_DIR) \
    -I$(NUKE_INCL_DIR)
#

CXXFLAGS += $(OPTIMIZE) $(INCLDIRS)
#CXXFLAGS += -std=c++0x

# enable unresolved symbol warnings:
LDFLAGS  += -Wl,-z,defs

# From $(NUKE_LOCATION)/Documentation/NDK/examples/Makefile:
CXXFLAGS += \
    -DUSE_GLEW
#

ifeq (32,$(strip $(bits)))
    CXXFLAGS += -m32 -DBits32_
else
    CXXFLAGS += -m64 -DBits64_
endif

ifdef LINUX
    CXXFLAGS += -DLINUX
    ifeq (32,$(strip $(bits)))
        CXXFLAGS += -DLINUX_32
    else
        CXXFLAGS += -DLINUX_64
    endif
endif
ifdef MBSD
    # Darwin ld treats undefined symbols as errors by default;
    # change to runtime resolution, like Linux.
    CXXFLAGS += -undefined dynamic_lookup
endif

ifeq (no,$(strip $(shared)))
    LIBS += $(OPENDCX_LIBS) $(OPENEXR_LIBS)
else
    LIBS += \
        -Wl,-rpath,$(OPENEXR_LIB_DIR) -L$(OPENEXR_LIB_DIR) $(OPENEXR_LIBS) \
        -Wl,-rpath,$(OPENDCX_LIB_DIR) -L$(OPENDCX_LIB_DIR) $(OPENDCX_LIBS)
endif #shared

LIBS += \
    -Wl,-rpath,$(NUKE_LOCATION) -L$(NUKE_LOCATION) -lDDImage \
    -ldl -lm -lz

#
ifdef LINUX
    LIBS += -lrt
    LIBS_RPATH += -lrt
endif

# Plugin source - each .cpp becomes a .so plugin
NUKE_PLUGIN_SRC_NAMES := \
    nuke/DeepMatte.cpp \
    nuke/DeepToImage.cpp \
    nuke/DeepTransform.cpp \
#

# Source code shared between plugins:
NUKE_SHARED_SRC_NAMES := \
    nuke/DDImageAdapter.cpp \
#

# Script files to install in $(NUKE_DESTDIR)/plugins
NUKE_PLUGIN_SCRIPT_NAMES := \
    nuke/DeepSurfaceType.tcl \
#

# Script files to install in $(NUKE_DESTDIR)/scripts
NUKE_SCRIPT_NAMES := \
#

# File to build dependecies for
DEPEND_FILES := \
    $(NUKE_PLUGIN_SRC_NAMES) \
    $(NUKE_SHARED_SRC_NAMES) \
#


NUKE_PLUGIN_OBJ_NAMES := \
    $(NUKE_SHARED_SRC_NAMES:.cpp=.o) \
    $(NUKE_PLUGIN_SRC_NAMES:.cpp=.o)
NUKE_PLUGIN_SO_NAMES := \
    $(NUKE_PLUGIN_SRC_NAMES:.cpp=.so)

#$(addprefix $(NUKE_DESTDIR)/,$(notdir $(NUKE_PLUGIN_SRC_NAMES:.cpp=.so)))


DEPEND := nuke_deps

# Get the list of dependencies that are newer than the current target,
# but limit the list to at most three entries.
list_deps = $(if $(wordlist 4,5,$(?F)),$(firstword $(?F)) and others,$(wordlist 1,3,$(?F)))

ALL_TARGETS := \
    $(DEPEND) \
    $(NUKE_PLUGIN_OBJ_NAMES) \
    $(NUKE_PLUGIN_SO_NAMES)
#

.SUFFIXES: .o .cpp

.PHONY: all clean depend install

all: depend

$(NUKE_PLUGIN_OBJ_NAMES): %.o: %.cpp
	@echo "-----------------------------------------------------------------"
	@echo "Compiling $@ because of $(call list_deps)"
	$(CXX) -c $(CXXFLAGS) -fPIC -o $@ $<

$(NUKE_PLUGIN_SO_NAMES): nuke/%.so: nuke/%.o $(NUKE_SHARED_SRC_NAMES:.cpp=.o)
	@echo "Linking $@ because of $(call list_deps)"
	$(CXX) $(LDFLAGS) $(CXXFLAGS) -fPIC -Wl,-Bsymbolic -shared -o $@ $^ $(LIBS)

install: $(NUKE_PLUGIN_SO_NAMES)
	@echo "Installing $<"
	if [ ! -d "$(NUKE_DESTDIR)/plugins" ]; then \
	    mkdir -p $(NUKE_DESTDIR)/plugins; \
	    echo "Created $(NUKE_DESTDIR)/plugins"; \
	fi
	cp -f $(NUKE_PLUGIN_SO_NAMES) $(NUKE_PLUGIN_SCRIPT_NAMES) $(NUKE_DESTDIR)/plugins/
	@echo "Copied $(NUKE_PLUGIN_SO_NAMES) $(NUKE_PLUGIN_SCRIPT_NAMES) to $(NUKE_DESTDIR)/plugins/"
	#if [ ! -d "$(NUKE_DESTDIR)/scripts" ]; then \
	#    mkdir -p $(NUKE_DESTDIR)/scripts; \
	#    echo "Created $(NUKE_DESTDIR)/scripts"; \
	#fi
	#cp -f $(NUKE_SCRIPT_NAMES) $(NUKE_DESTDIR)/scripts/
	#@echo "Copied $(NUKE_SCRIPT_NAMES) to $(NUKE_DESTDIR)/scripts/"


# TODO: This accumulates all source file dependencies into a single file
# containing a rule for each *.o file.  Consider generating a separate
# dependency file for each *.o file instead.
$(DEPEND): $(DEPEND_FILES)
	@echo "Generating dependencies because of $(call list_deps)"
	$(RM) $(DEPEND)
	for f in $(DEPEND_FILES); \
		do $(CXX) $(CXXFLAGS) -O0 -MM $$f -MT `echo $$f | sed 's%\.[^.]*%.o%'` \
			-I $(NUKE_INCL_DIR) >> $(DEPEND); \
	done

depend: $(DEPEND)

clean:
	$(RM) $(ALL_TARGETS) $(DEPEND) $(NUKE_OBJ_NAMES)

ifneq (,$(strip $(wildcard $(DEPEND))))
    include $(DEPEND)
endif

