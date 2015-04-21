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
 # $Id: flagdefs.mk,v 1.3 2001/07/13 22:46:01 ulucs Exp $
 #
 # Makefile definitions for compiler and linker flags
 #
 # Created       : Uluc Saranli, 11/16/2000
 # Last Modified : Uluc Saranli, 06/27/2001
 #
 ######################################################################

UNAME = $(shell uname)

ifeq ($(UNAME), Linux)
# Definitions for Linux compilation

DEFINES = -D_LINUX_ -D$(RHEX_HARDWARE)
INC      = -I$(RHEX_DIR)/include -I$(SIMSECT_DIR)/include \
	-I$(SIMSECT_DIR)/systems/hexapod $(AUXFLAGS) $(DEFINES) 
CXXFLAGS = -Wall -g -I$(RHEX_DIR)/include -I$(SIMSECT_DIR)/include \
	-I$(SIMSECT_DIR)/systems/hexapod  $(DEFINES) $(AUXFLAGS)
LDFLAGS  = -L$(RHEX_DIR)/lib -L$(SIMSECT_DIR)/lib $(AUXFLAGS)
LDLIBS   = -lm

endif

ifeq ($(UNAME),QNX)
# Definitions for QNX compilation

DEFINES = -D_QNX4_ -D$(RHEX_HARDWARE)
INC      = -I$(RHEX_DIR)/include $(DEFINES) $(AUXFLAGS)
CXXFLAGS = -wx -I$(RHEX_DIR)/include $(DEFINES) -Ox -Oa -Ot -Om  -g1 $(AUXFLAGS)
LDFLAGS  = -L$(RHEX_DIR)/lib "-Wl, OPTION PRIVILEGE=1"  -g1 $(AUXFLAGS)
LDLIBS   = 

endif
