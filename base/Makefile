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
 # $Id: Makefile,v 1.4 2001/08/09 20:50:48 usofrob Exp $
 #
 # The Makefile for the base RHex library
 #
 # Created       : Uluc Saranli, 11/16/2000
 # Last Modified : Uluc Saranli, 06/27/2001
 #
 ######################################################################

############################################
# Modify the following segment only.       #
############################################

# This line specifies the name of the library in which all the object
# files for the sources below will be collected into.
LIBRARY = libbase.a

# This line specifies the source files that will becompiled to form
# the library
SOURCES = \
	Module.cc \
	ModuleManager.cc \
	StateMachine.cc \
	AnalogOutput.cc \
	SpeedFilter.cc \
	EncoderReader.cc \
	StallSensor.cc \
	PositionControl.cc \
	PulseWidth.cc \
	Tokenizer.cc \
	SymbolTable.cc \
	DataLogger.cc \
	ExperimentRunner.cc \
	RemoteControl.cc \
	VirtualInput.cc \
	sysutil.cc \
	Buffer.cc \
	Filter.cc \
	Polynomial.cc \
	DiscreteSystem.cc \
	PolyRatio.cc \
	ExtProfiler.cc \
	ExpRunner.cc \
    Floats.cc \
    Strings.cc 

# This line specifies any additional flags that you want to feed the
# compiler. It is usually used to specify additional include
# directories through -I and additional link directories through -L
AUXFLAGS = 

#################################################
# The rest of this file should not be modified  #
#################################################

ifndef RHEX_DIR
all:
	@echo
	@echo "RHEX_DIR environment variable undefined!"
else

include $(RHEX_DIR)/tools/libtargets.mk

endif
