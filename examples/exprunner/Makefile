 #
 # This file is part of RHexLib, 
 #
 # Copyright (c) 2001 The University of Michigan, its Regents,
 # Fellows, Employees and Agents. All rights reserved, and distributed as
 # free software under the following license.
 # 
 #  Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are
 # met:
 # 
 # 1) Redistributions of source code must retain the above copyright
 # notice, this list of conditions, the following disclaimer and the
 # file called "CREDITS" which accompanies this distribution.
 # 
 # 2) Redistributions in binary form must reproduce the above copyright
 # notice, this list of conditions, the following disclaimer and the file
 # called "CREDITS" which accompanies this distribution in the
 # documentation and/or other materials provided with the distribution.
 # 
 # 3) Neither the name of the University of Michigan, Ann Arbor or the
 # names of its contributors may be used to endorse or promote products
 # derived from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 # A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 # LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ######################################################################
 # $Id: Makefile,v 1.1 2001/08/12 18:57:33 ulucs Exp $
 #
 # The Makefile for the RHexLib example executables
 #
 # Created       : Uluc Saranli, 11/16/2000
 # Last Modified : Uluc Saranli, 06/27/2001
 #
 ######################################################################

UNAME = $(shell uname)

############################################
# Modify the following segment only.       #
############################################

# This line defines the name of the executables. Note that we assume
# the presence of an associated .cc file as well.
EXEC = walk_test

# This line specifies all the other source files that need to be
# compiled and linked together with the executable
SOURCES = WalkExp.cc

# This line specifies any additional flags that you want to feed the
# compiler. It is usually used to specify additional include
# directories through -I and additional link directories through -L
AUXFLAGS = 

# This line specifies any additional libraries that you want to link
# your executable against. Note that you MUST specify the full path of
# the library in question.
AUXLIBS = 

# The following sections set the variable LIBS based on which hardware
# platform is currently being uised. The LIBS variable contains ONLY
# the names of the libraries, without the path. The Makefile tools
# append the path automatically.
ifeq ($(UNAME), QNX)

ifeq ($(RHEX_HARDWARE), _MICHIGAN_)

LIBS = librhex.a libvirtualhw.a libmichiganhw.a libcommonhw.a libbase.a

endif

ifeq ($(RHEX_HARDWARE), _MCGILL_)

LIBS = librhex.a libvirtualhw.a libmcgillhw.a libcommonhw.a libbase.a

endif

endif

ifeq ($(UNAME), Linux)

LIBS = librhex.a libvirtualhw.a libsimsecthw.a libcommonhw.a libbase.a
#For SimSect, we will need some additional libraries
AUXLIBS += $(SIMSECT_DIR)/lib/libhexapod.a $(SIMSECT_DIR)/lib/libengine.a 

endif

#################################################
# The rest of this file should not be modified  #
#################################################

ifndef RHEX_DIR
all:
	@echo
	@echo "RHEX_DIR environment variable undefined!"
else

include $(RHEX_DIR)/tools/bintargets.mk

endif
