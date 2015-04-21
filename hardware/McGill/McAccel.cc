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
 * $Id: McAccel.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Low level hardware interface to the accelerometers of the McGill
 * version of RHex
 *
 * Created       : Uluc Saranli, 01/23/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <math.h>
#include "ModuleManager.hh"
#include "PulseWidth.hh"
#include "McGillHW.hh"
#include "McComponents.hh"

#define A1_ADC_CHAN 13
#define A2_ADC_CHAN 14
#define A3_ADC_CHAN 15
#define G1_ADC_CHAN 10
#define G2_ADC_CHAN 11
#define G3_ADC_CHAN 12

// McAccel::McAccel : Class constructor.
// Creates and initializes the PulseWidth modules.

McAccel::McAccel( Hardware *hw_in ) {

  int i;
  float gainD[3] = { 22219.70555, 22219.70555, 22219.70555 };
  float offsetD[3] = { 1.9498e-3, 1.9498e-3, 1.9498e-3 };
  Floats temp;
  char msg[128];

  hw = hw_in;
  
  MMMessage( "Initializing Accelerometer components..." );

  // Read the affine accelerometer map parameters from the main symbol table
  temp = MMGetArraySymbol( "accelerometer_voltage_offsets" );
  for ( i = 0; i < temp.getCount(); i++ ) 
    offsets[ i ] = temp.get(i);
  for ( i = temp.getCount(); i < 3; i++ )
    offsets[ i ] = offsetD[i];

  temp = MMGetArraySymbol( "accelerometer_voltage_gains" );
  for ( i = 0; i < temp.getCount(); i++ ) 
    gains[ i ] = temp.get(i);
  for ( i = temp.getCount(); i < 3; i++ )
    gains[ i ] = gainD[ i ];


  sprintf( msg, "\n  x:{%1.2f,%1.2f}, y:{%1.2f,%1.2f}, z:{%1.2f,%1.2f}.", 
           offsets[0], gains[0], offsets[1], gains[1], offsets[2], gains[2]);
  MMMessage( msg );
}

// MAccel::~MAccel : Class destructor.
McAccel::~McAccel( void ) {
}

// MAccel::read : Reads in the value of a continuous dial.
float McAccel::read( Axis a ) {
  float voltage;
  float offset, gain, val;

  // Decide on the axis and retrieve the pulse width and the 
  // calibration parameters
  if ( a == AXIS_X ) 
  {

    voltage = hw->analogIO->read( A1_ADC_CHAN );
    offset = offsets[ 0 ];
    gain = gains[ 0 ];

  } 
  else if ( a == AXIS_Y ) 
  {

    voltage = hw->analogIO->read( A2_ADC_CHAN );
    offset = offsets[ 1 ];
    gain = gains[ 1 ];

  } 
  else if ( a == AXIS_Z ) 
  {

    voltage = hw->analogIO->read( A3_ADC_CHAN );
    offset = offsets[ 2 ];
    gain = gains[ 2 ];
  } 

  // Compute the acceleration
  val = gain * ( voltage - offset );

  return val;
}

void McAccel::getInfo( Axis axis, float *resolution, float *limit ) {

  // The resolution is a function of the timer frequency and the 
  // accelerometer range and period.

  // For 2G version
  *resolution = 43.0e-3;
  *limit = 2.0;
}
