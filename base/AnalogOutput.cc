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
 * $Id: AnalogOutput.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The Analog Output Module
 *
 * This file implements the functions defined in AnalogOutput.hh
 *
 * Created       : Uluc Saranli, 10/25/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include "ModuleManager.hh"
#include "StdModules.hh"
#include "AnalogOutput.hh"
#include "basicmath.hh"

// AnalogOutput::AnalogOutput: Class constructor
AnalogOutput::AnalogOutput ( int index )
  : Module ( ANALOGOUTPUT_NAME, index, true, false ) {

  Hardware *hw = MMGetHardware();

  if ( index < 0 || index >= AOUT_MAX_INDEX )
    MMFatalError ( "AnalogOutput::AnalogOutput", "Index out of range!" );

  if ( hw->analogIO == NULL)
    MMFatalError ( "AnalogOutput::AnalogOutput", "Invalid hardware object!" );

  value = 0;

  analogIO = hw->analogIO;
  if ( analogIO == NULL )
    MMFatalError( "AnalogOutput::AnalogOutput",
                  "Analog I/O hardware component not supported!" );

  analogIO->outputRange( getIndex(), & minOut, & maxOut );
}

// AnalogOutput::update: Updates an analog output value.
void AnalogOutput::update () {

  // printf ( "Writing analog channel %i, value: %f\n", getIndex(), value );
  
  // Write the actual voltage to the hardware
  analogIO->write( getIndex(), value );

}

// AnalogOutput::setValue: Sets the analog output value in volts
void AnalogOutput::setValue ( float v ) { 
  
  value = saturate( v, minOut, maxOut );

};


