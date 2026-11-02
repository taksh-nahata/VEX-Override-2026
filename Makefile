################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
# -g0 turns off debug info (common.mk adds -g by default). The V5 has no
# debugger attached to it, so it only made objects bigger and rebuilds slower.
EXTRA_CFLAGS=-g0
EXTRA_CXXFLAGS=-g0 -Wno-deprecated-enum-enum-conversion

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS!
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported
TEMPLATE_FILES=$(INCDIR)/**/*.h $(INCDIR)/**/*.hpp

.DEFAULT_GOAL=quick

# Compile independent .cpp files in parallel instead of one at a time.
MAKEFLAGS+=-j8

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

# common.mk defines the color/status strings with a plain `=` around
# $(shell printf ...), so make spawns a new shell process every time one is
# used -- a couple dozen per build, and process spawns are slow on Windows.
# Re-assigning them with := evaluates each once (~2s faster per build).
# Order matters: colors first, then the strings built from them.
NO_COLOR    := $(NO_COLOR)
OK_COLOR    := $(OK_COLOR)
ERROR_COLOR := $(ERROR_COLOR)
WARN_COLOR  := $(WARN_COLOR)
STEP_COLOR  := $(STEP_COLOR)
OK_STRING    := $(OK_STRING)
DONE_STRING  := $(DONE_STRING)
ERROR_STRING := $(ERROR_STRING)
WARN_STRING  := $(WARN_STRING)
