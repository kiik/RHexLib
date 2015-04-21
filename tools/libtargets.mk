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
 # $Id: libtargets.mk,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 #
 # Makefile utility to specify source files for a library target.
 #
 # Created       : Uluc Saranli, 11/16/2000
 # Last Modified : Uluc Saranli, 06/27/2001
 #
 ######################################################################

ifndef RHEX_HARDWARE
all:
	@echo
	@echo "Error: RHEX_HARDWARE environment variable undefined!"
	@echo "Set RHEX_HARDWARE to the appropriate platform ( _MICHIGAN_, _MCGILL_ etc. )!"
else

include $(RHEX_DIR)/tools/tooldefs.mk
include $(RHEX_DIR)/tools/flagdefs.mk

# Derived parameters.

OBJECTS = $(SOURCES:.cc=.o)

# Standard targets.

$(LIBRARY) : $(OBJECTS)
	$(AR) rv $@ $?
#	$(RANLIB) $@

depend dep:
	$(ECHO) > Makefile.dep
	$(MAKEDEP) -fMakefile.dep -Y -- $(INC) -- `find . -name '*.cc'` >& /dev/null
#	for i in `find . -name '*.cc'`; do $(CMM) $(INC) $$i; done > Makefile.dep

wc :
	@$(WC) -l *.cc *.hh Makefile | $(SORT) -r -n

clean:
	$(RM) -rf $(OBJECTS) $(LIBRARY) *~ *.err
	$(RM) -rf Makefile.dep.bak

cleanobj:
	$(RM) -rf $(OBJECTS) *~ *.err
	$(RM) -rf Makefile.dep.bak


### Dependencies

-include Makefile.dep

endif
