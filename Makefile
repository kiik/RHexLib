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
 # $Id: Makefile,v 1.9 2001/08/14 03:33:44 ulucs Exp $
 #
 # The Makefile for the Module Manager
 #
 # Created       : Uluc Saranli, 11/16/2000
 # Last Modified : Uluc Saranli, 06/27/2001
 #
 ######################################################################

ifndef RHEX_DIR
all:
	@echo
	@echo "Error: RHEX_DIR environment variable undefined!"
	@echo "Please set RHEX_DIR to the base directory of RHexLib installation!"
else
ifndef RHEX_HARDWARE
all:
	@echo
	@echo "Error: RHEX_HARDWARE environment variable undefined!"
	@echo "Please set RHEX_HARDWARE to name of the platform ( MICHIGAN, MCGILL, UT etc. )!"
else

include $(RHEX_DIR)/tools/tooldefs.mk

DIRNAME = RHexLib
VERSION = 1.0
PACKAGENAME = RHexLib_core
BINDIR = examples examples/quickstart examples/esp examples/exprunner
DOCDIR = doc

LIBS = 	base/libbase.a rhex/librhex.a

# Determine which hardware libraries to compile based on the current system
ifeq ($(UNAME), QNX)
ifeq ($(RHEX_HARDWARE), _MICHIGAN_)
HWLIBS = hardware/virtual/libvirtualhw.a \
	hardware/common/libcommonhw.a \
	hardware/UofM/libmichiganhw.a
endif
ifeq ($(RHEX_HARDWARE), _MCGILL_)
HWLIBS = hardware/virtual/libvirtualhw.a \
	hardware/common/libcommonhw.a \
	hardware/McGill/libmcgillhw.a
endif
ifeq ($(RHEX_HARDWARE), _UT_)
HWLIBS = hardware/virtual/libvirtualhw.a \
	hardware/common/libcommonhw.a \
	hardware/McGill/libuthw.a
endif

endif

ifeq ($(UNAME), Linux)
HWLIBS = hardware/virtual/libvirtualhw.a \
	hardware/common/libcommonhw.a \
	hardware/SimSect/libsimsecthw.a
endif

### Derived parameters

CLEAN_LIST = $(LIBS:.a=.clean) $(HWLIBS:.a=.clean)

CLEANOBJ_LIST = $(LIBS:.a=.cleanobj) $(HWLIBS:.a=.cleanobj)

FILES_LIST = $(LIBS:.a=.dir) $(HWLIBS:.a=.dir)

DEPEND_LIST = $(LIBS:.a=.dep) $(HWLIBS:.a=.dep)

include $(RHEX_DIR)/tools/flagdefs.mk


### Standard targets.

.PHONY: libs exec $(BINDIR) all
.PHONY: clean $(CLEAN_LIST) cleanobj $(CLEANOBJ_LIST)
.PHONY: wc tarball depend dep bindepend

libs : $(LIBS) $(HWLIBS)

all : libs exec

exec :  $(BINDIR)

$(BINDIR) :
	@$(ECHO)
	@$(CD) $@ ; $(MAKE)

$(LIBS) $(HWLIBS): FORCE
	@$(ECHO)
	@$(CD) $(@D) ; $(MAKE) $(@F)
	@$(LN) ../$(@) lib

clean : $(CLEAN_LIST)
	for dir in $(BINDIR); do \
	  $(MAKE) -C $$dir clean; \
	done
	@$(CD) $(DOCDIR); $(MAKE) realclean
	$(RM) -f lib/*.a

$(CLEAN_LIST): FORCE
	@$(ECHO)
	@$(CD) $(@D); $(MAKE) clean

cleanobj : $(CLEANOBJ_LIST)
	for dir in $(BINDIR); do \
	  $(MAKE) -C $$dir cleanobj; \
	done

$(CLEANOBJ_LIST): FORCE
	@$(ECHO)
	@$(CD) $(@D); $(MAKE) cleanobj

wc :
	@$(WC) -l `find . -name Makefile` `find . -name '*.cc'` `find . -name '*.hh'` | $(SORT) -r -n

tarball : clean
	@$(RM) -f ../$(PACKAGENAME)-$(VERSION).tar.gz
	find . -name Makefile.dep | xargs $(RM) -f
	find . -name Makefile.dep.bak | xargs $(RM) -f
	@$(CD) ..; find $(DIRNAME) -name CVS > cvsfiles.list
	$(CD) ..; $(TAR) cvf $(PACKAGENAME)-$(VERSION).tar $(DIRNAME) -Xcvsfiles.list; $(GZIP) $(PACKAGENAME)-$(VERSION).tar
	$(CD) ..; $(RM) -f cvsfiles.list

depend dep : $(DEPEND_LIST) bindepend

$(DEPEND_LIST) : FORCE
	@$(ECHO)
	@$(CD) $(@D); $(MAKE) depend

bindepend:
	for dir in $(BINDIR); do \
	  $(MAKE) -C $$dir depend; \
	done

FORCE :

endif
endif
