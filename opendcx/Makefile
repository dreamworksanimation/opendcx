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
# Makefile for OpenDCX library
#


#---------------------------------------------------------------------------
# DWA local build settings, remove before publishing:
CXX              := /rel/lang/gcc/gcc412/bin/g++412
LDFLAGS          := -Wl,-rpath,/rel/lang/gcc/gcc412/lib64 -L/rel/lang/gcc/gcc412/lib64
OPENEXR_INCL_DIR := /rel/folio/openexr/openexr-2.2.0.d.w.a.5.0-latest/include
OPENEXR_LIB_DIR  := /rel/folio/openexr/openexr-2.2.0.d.w.a.5.0-latest/lib
#---------------------------------------------------------------------------


# See INSTALL for a list of requirements.
#
# Targets:
#   lib                 the OpenDCX library
#
#   dcx_print           command-line tool to inspect OpenDCX files
#   dcx_render          command-line tool to ray-trace OpenDCX files
#   dcx_test            unit tests for the OpenDCX library
#
#   all                 [default target] all of the above
#   install             install all of the above
#                       into subdirectories of DESTDIR
#   depend              recompute source file header dependencies
#   clean               delete generated files from the local directory
#   test                run tests
#
# Options:
#   shared=yes          produce shared OpenDCX libraries (default = yes)
#   static=yes          produce static OpenDCX libraries (default = no)
#                       the version # is added before .a to allow linking
#                       against an explicit version
#   debug=yes           build with debugging symbols and without optimization
#   verbose=yes         run commands in verbose mode


#
# The following variables must be defined, either here or on the command line
# (e.g., "make install DESTDIR=/usr/local"):
#

# The directory into which to install libraries, executables and header files
DESTDIR := /tmp/OpenDCX

# OpenDCX requires OpenEXR 2.1 or greater.
ifndef OPENEXR_INCL_DIR
    # The parent directory of the OpenEXR header directory
    OPENEXR_INCL_DIR := /usr/include
endif
ifndef OPENEXR_LIB_DIR
    # The directory containing IlmImf & IlmImfUtil libs
    OPENEXR_LIB_DIR := /usr/lib64
endif
OPENEXR_LIBS := -lIlmImfUtil -lIlmImf

#
# Ideally, users shouldn't need to change anything below this line.
#

SHELL = /bin/bash

# Turn off implicit rules for speed.
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

ifeq (yes,$(strip $(debug)))
    OPTIMIZE := -g
else
    OPTIMIZE := -O3 -DNDEBUG
endif

ifeq (yes,$(strip $(verbose)))
    QUIET_TEST := -v
else
    QUIET_TEST := $(QUIET)
endif

INCLDIRS := -I . -I .. -isystem $(OPENEXR_INCL_DIR)

#CXXFLAGS += -std=c++0x

#CXXFLAGS += -pthread $(OPTIMIZE) $(INCLDIRS)
CXXFLAGS += $(OPTIMIZE) $(INCLDIRS)

# Default to building shared if shared and static not defined.
ifndef shared
    ifndef static
        shared = yes
    endif
endif

LIBS := \
    -L$(OPENEXR_LIB_DIR) $(OPENEXR_LIBS) \
#    -ldl -lm -lz \
#
LIBS_RPATH := \
    -Wl,-rpath,$(OPENEXR_LIB_DIR) -L$(OPENEXR_LIB_DIR) $(OPENEXR_LIBS) \
#    -ldl -lm -lz \
#

ifdef LINUX
    LIBS += -lrt
    LIBS_RPATH += -lrt
endif

INCLUDE_NAMES := \
        DcxAPI.h \
        DcxChannelAlias.h \
        DcxChannelDefs.h \
        DcxChannelSet.h \
        DcxDeepAccumulator.h \
        DcxDeepImageTile.h \
        DcxDeepPixel.h \
        DcxDeepTile.h \
        DcxDeepTransform.h \
        DcxImageFormat.h \
        DcxPixel.h \
        DcxSpMask.h \
        version.h
#

SRC_NAMES := \
    DcxChannelSet.cpp \
    DcxDeepAccumulator.cpp \
    DcxDeepImageTile.cpp \
    DcxDeepPixel.cpp \
    DcxDeepTile.cpp \
    DcxDeepTransform.cpp \
    DcxImageFormat.cpp
#

UNITTEST_INCLUDE_NAMES := \
#

UNITTEST_SRC_NAMES := \
#

CMD_INCLUDE_NAMES := \
#

CMD_SRC_NAMES := \
#    cmd/opendcx_print/main.cpp \
#    cmd/opendcx_render/main.cpp \
#    cmd/opendcx_view/main.cpp \
#

HEADER_SUBDIRS := $(dir $(INCLUDE_NAMES))

ALL_INCLUDE_FILES := \
    $(INCLUDE_NAMES) \
    $(UNITTEST_INCLUDE_NAMES) \
    $(CMD_INCLUDE_NAMES)
#
SRC_FILES := \
    $(SRC_NAMES) \
    $(UNITTEST_SRC_NAMES) \
    $(CMD_SRC_NAMES)
#
ALL_SRC_FILES := $(SRC_FILES)

OBJ_NAMES := $(SRC_NAMES:.cpp=.o)
UNITTEST_OBJ_NAMES := $(UNITTEST_SRC_NAMES:.cpp=.o)

LIB_MAJOR_VERSION=$(shell grep 'define OPENDCX_VERSION_MAJOR ' \
    version.h | sed 's/[^0-9]*//g')
LIB_MINOR_VERSION=$(shell grep 'define OPENDCX_VERSION_MINOR ' \
    version.h | sed 's/[^0-9]*//g')
LIB_PATCH_VERSION=$(shell grep 'define OPENDCX_VERSION_PATCH ' \
    version.h | sed 's/[^0-9]*//g')

LIB_VERSION=$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION).$(LIB_PATCH_VERSION)
SO_VERSION=$(LIB_MAJOR_VERSION).$(LIB_MINOR_VERSION)

LIBOPENDCX_NAME=libOpenDCX

# shared lib names:
ifndef MBSD
    LIBOPENDCX_SHARED_NAME := $(LIBOPENDCX_NAME).so
    LIBOPENDCX_SHARED := $(LIBOPENDCX_NAME).so.$(LIB_VERSION)
    LIBOPENDCX_SONAME := $(LIBOPENDCX_NAME).so.$(SO_VERSION)
    LIBOPENDCX_SONAME_FLAGS := -Wl,-soname,$(LIBOPENDCX_SONAME)
else
    LIBOPENDCX_SHARED_NAME := $(LIBOPENDCX_NAME).dylib
    LIBOPENDCX_SHARED := $(LIBOPENDCX_NAME).$(LIB_VERSION).dylib
    LIBOPENDCX_SONAME := $(LIBOPENDCX_NAME).$(SO_VERSION).dylib
    LIBOPENDCX_SONAME_FLAGS := -Wl,-install_name,$(DESTDIR)/lib/$(LIBOPENDCX_SONAME)
endif
LIBOPENDCX_RPATH := -Wl,-rpath,$(DESTDIR)/lib

# static lib names:
# The version # is put before .a to allow linking against an explicit version.
LIBOPENDCX_STATIC_NAME := $(LIBOPENDCX_NAME).a
LIBOPENDCX_STATIC := $(LIBOPENDCX_NAME)-$(LIB_VERSION).a


ALL_PRODUCTS := \
    $(DEPEND) \
    $(LIBOPENDCX_SHARED_NAME) \
    $(LIBOPENDCX_SHARED) \
    $(LIBOPENDCX_SONAME) \
    $(LIBOPENDCX_STATIC_NAME) \
    $(LIBOPENDCX_STATIC)
#    dcx_test \
#    dcx_print \
#    dcx_render \
#    dcx_view


BUILD_PRODUCTS := \
    $(DEPEND) \
#    dcx_test \
#    dcx_print \
#    dcx_render \
#    dcx_view

ifeq (yes,$(strip $(shared)))
    BUILD_PRODUCTS += \
        $(LIBOPENDCX_SHARED_NAME) \
        $(LIBOPENDCX_SHARED) \
        $(LIBOPENDCX_SONAME)
endif

ifeq (yes,$(strip $(static)))
    BUILD_PRODUCTS += \
        $(LIBOPENDCX_STATIC_NAME) \
        $(LIBOPENDCX_STATIC)
endif


DEPEND := dependencies

# Get the list of dependencies that are newer than the current target,
# but limit the list to at most three entries.
list_deps = $(if $(wordlist 4,5,$(?F)),$(firstword $(?F)) and others,$(wordlist 1,3,$(?F)))

#

.SUFFIXES: .o .cpp

.PHONY: all clean depend install lib test

.cpp.o:
	@echo "Building $@ because of $(call list_deps)"
	$(CXX) -c $(CXXFLAGS) -fPIC -o $@ $<

#all: lib dcx_print dcx_render dcx_test depend
all: lib depend

$(OBJ_NAMES): %.o: %.cpp
	@echo "Building $@ because of $(call list_deps)"
	@echo "$(BUILD_PRODUCTS)"
	$(CXX) -c -DOPENDCX_PRIVATE $(CXXFLAGS) -fPIC -o $@ $<

ifeq (yes,$(strip $(shared)))
# Build shared library
lib: $(LIBOPENDCX_SHARED_NAME) $(LIBOPENDCX_SONAME)

$(LIBOPENDCX_SHARED_NAME): $(LIBOPENDCX_SHARED)
	ln -f -s $< $@

$(LIBOPENDCX_SONAME): $(LIBOPENDCX_SHARED)
	ln -f -s $< $@

$(LIBOPENDCX_SHARED): $(OBJ_NAMES)
	@echo "Building $@ because of $(list_deps)"
	$(CXX) $(LDFLAGS) $(CXXFLAGS) -shared -o $@ $^ $(LIBS_RPATH) $(LIBOPENDCX_SONAME_FLAGS)
endif #shared

ifeq (yes,$(strip $(static)))
# Build static library
lib: $(LIBOPENDCX_STATIC) $(LIBOPENDCX_STATIC_NAME)

$(LIBOPENDCX_STATIC): $(OBJ_NAMES)
	@echo "Building $@ because of $(list_deps)"
	$(AR) cr $@ $^

$(LIBOPENDCX_STATIC_NAME): $(LIBOPENDCX_STATIC)
	ln -f -s $< $@
endif #static


#dcx_print: $(LIBOPENDCX) cmd/opendcx_print/main.cpp
#	@echo "Building $@ because of $(list_deps)"
#	$(CXX) $(CXXFLAGS) -o $@ cmd/opendcx_print/main.cpp -I . \
#	    $(LIBOPENDCX_RPATH) -L$(CURDIR) $(LIBOPENDCX) \
#	    $(LIBS_RPATH) $(CONCURRENT_MALLOC_LIB)

#dcx_render: $(LIBOPENDCX) cmd/opendcx_render/main.cpp
#	@echo "Building $@ because of $(list_deps)"
#	$(CXX) $(CXXFLAGS) -o $@ cmd/opendcx_render/main.cpp -I . \
#	    -isystem $(OPENEXR_INCL_DIR) -isystem $(ILMBASE_INCL_DIR) \
#	    -Wl,-rpath,$(OPENEXR_LIB_DIR) -L$(OPENEXR_LIB_DIR) $(OPENEXR_LIB) \
#	    -Wl,-rpath,$(ILMBASE_LIB_DIR) -L$(ILMBASE_LIB_DIR) $(ILMBASE_LIB) \
#	    $(LIBOPENDCX_RPATH) -L$(CURDIR) $(LIBOPENDCX) \
#	    $(LIBS_RPATH) $(CONCURRENT_MALLOC_LIB)

# Create an opendcx_viewer/ symlink to the viewer/ subdirectory,
# to mirror the DWA directory structure.
#opendcx_viewer:
#	ln -f -s viewer opendcx_viewer

$(UNITTEST_OBJ_NAMES): %.o: %.cpp
	@echo "Building $@ because of $(list_deps)"
	$(CXX) -c $(CXXFLAGS) -fPIC -o $@ $<

#ifneq (,$(strip $(CPPUNIT_INCL_DIR)))
#dcx_test: $(LIBOPENDCX) $(UNITTEST_OBJ_NAMES)
#	@echo "Building $@ because of $(list_deps)"
#	$(CXX) $(CXXFLAGS) -o $@ $(UNITTEST_OBJ_NAMES) \
#	    -Wl,-rpath,$(LIBOPENDCX_RPATH) -L$(CURDIR) $(LIBOPENDCX) $(LIBS_RPATH)
#
#test: lib dcx_test
#	@echo "Testing $(LIBOPENDCX_NAME)"
#	export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(CURDIR); ./dcx_test $(QUIET_TEST)
#else
#dcx_test:
#	@echo "$@"': $$(CPPUNIT_INCL_DIR) is undefined'
#test:
#	@echo "$@"': $$(CPPUNIT_INCL_DIR) is undefined'
#endif

#install: lib dcx_print dcx_render dcx_view
install: lib
	mkdir -p $(DESTDIR)/include/OpenDCX
	@echo "Created $(DESTDIR)/include/OpenDCX"
	pushd $(DESTDIR)/include/OpenDCX > /dev/null; \
	    mkdir -p $(HEADER_SUBDIRS); popd > /dev/null
	for f in $(INCLUDE_NAMES); \
	    do cp -f $$f $(DESTDIR)/include/OpenDCX/$$f; done
	@echo "Copied header files to $(DESTDIR)/include"
	@#
	mkdir -p $(DESTDIR)/lib
	@echo "Created $(DESTDIR)/lib/"
	if [ -f $(LIBOPENDCX_SHARED) ]; then \
	    cp -f $(LIBOPENDCX_SHARED) $(DESTDIR)/lib; \
	fi
	pushd $(DESTDIR)/lib > /dev/null; \
	    if [ -f $(LIBOPENDCX_SHARED) ]; then \
	        ln -f -s $(LIBOPENDCX_SHARED) $(LIBOPENDCX_SHARED_NAME); \
	        ln -f -s $(LIBOPENDCX_SHARED) $(LIBOPENDCX_SONAME); \
	    fi; \
	    popd > /dev/null
	if [ -f $(LIBOPENDCX_STATIC) ]; then \
	    cp -f $(LIBOPENDCX_STATIC) $(DESTDIR)/lib; \
	fi
	pushd $(DESTDIR)/lib > /dev/null; \
	    if [ -f $(LIBOPENDCX_STATIC) ]; then \
	        ln -f -s $(LIBOPENDCX_STATIC) $(LIBOPENDCX_STATIC_NAME); \
	    fi; \
	    popd > /dev/null
	@echo "Copied libOpenDCX to $(DESTDIR)/lib/"
	#@#
	#mkdir -p $(DESTDIR)/bin
	#@echo "Created $(DESTDIR)/bin/"
	#cp -f dcx_print $(DESTDIR)/bin
	#@echo "Copied dcx_print to $(DESTDIR)/bin/"
	#cp -f dcx_render $(DESTDIR)/bin
	#@echo "Copied dcx_render to $(DESTDIR)/bin/"
	#if [ -f dcx_view ]; \
	#then \
	#    cp -f dcx_view $(DESTDIR)/bin; \
	#    echo "Copied dcx_view to $(DESTDIR)/bin/"; \
	#fi

# TODO: This accumulates all source file dependencies into a single file
# containing a rule for each *.o file.  Consider generating a separate
# dependency file for each *.o file instead.
$(DEPEND): $(ALL_INCLUDE_FILES) $(ALL_SRC_FILES)
	@echo "Generating dependencies because of $(list_deps)"
	$(RM) $(DEPEND)
	for f in $(SRC_NAMES) $(CMD_SRC_NAMES); \
	    do $(CXX) $(CXXFLAGS) -O0 \
	        -MM $$f -MT `echo $$f | sed 's%\.[^.]*%.o%'` >> $(DEPEND); \
	done

depend: $(DEPEND)

clean:
	$(RM) $(OBJ_NAMES) $(ALL_PRODUCTS)
	$(RM) $(UNITTEST_OBJ_NAMES)

ifneq (,$(strip $(wildcard $(DEPEND))))
    include $(DEPEND)
endif

