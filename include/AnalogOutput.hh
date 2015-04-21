/*
 * This file is part of RHexLib, 
 *
 * Copyright (c) 2001 The University of Michigan, its Regents,
 * Fellows, Employees and Agents. All rights reserved, and distributed as
 * free software under the following license.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1) Redistributions of source code must retain the above copyright
 * notice, this list of conditions, the following disclaimer and the
 * file called "CREDITS" which accompanies this distribution.
 * 
 * 2) Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions, the following disclaimer and the file
 * called "CREDITS" which accompanies this distribution in the
 * documentation and/or other materials provided with the distribution.
 * 
 * 3) Neither the name of the University of Michigan, Ann Arbor or the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*********************************************************************
 * AnalogOutput.hh
 *
 * The Analog Output Module
 *
 * This file contains the definition of the AnalogOutput module class 
 * and all the associated definitions. The member function definitions
 * are in AnalogOutput.c
 *
 *  The analog output module periodically sets the output value of 
 * the corresponding DAC hardware.
 *
 * Created       : Uluc Saranli, 10/25/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _ANALOGOUTPUT_HH
#define _ANALOGOUTPUT_HH

#include "ModuleManager.hh"
#include "Hardware.hh"

// Module specific constants ---------------------------------------

// Maximum number of separate analog outputs
#define AOUT_MAX_INDEX      8

// The AnalogOutput class ------------------------------------------
class AnalogOutput : public Module {

public:
  AnalogOutput ( int index );

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void );

  // Utilities to read and set analog output values. Note that this
  // interface assumes a 0-10V output range and the values set and
  // returned are in volts.
  void      setValue ( float v );
  float     getValue ( void ) { return value; };

private:
  // Analog output value in DAC digital counts
  float value;

  // Range of the current analog output hardware
  float minOut, maxOut;

  // Access to the analog IO hardware
  AnalogHW      *analogIO;

};


#endif

