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

/** @file UTAccel.cpp
 *  @brief Low level hardware interface to the accelerometers of the University of Tartu version of RHex
 *
 *  @author Meelik Kiik
 *  @version 0.1
 *  @date 21/04/2015 16:00
 *
 *  Created at 21/04/2015 16:00
 */

#include <math.h>
#include "ModuleManager.hh"
#include "PulseWidth.hh"

#include "TartuUniHW.h"
#include "UTComponents.h"


static PulseWidth *pwm[3];

// UTAccel::UTAccel : Class constructor.
// Creates and initializes the PulseWidth modules.
UTAccel::UTAccel( Hardware *hw ) {

  int i;
  float gainD[3] = { 22219.70555, 22219.70555, 22219.70555 };
  float offsetD[3] = { 1.9498e-3, 1.9498e-3, 1.9498e-3 };
  Floats temp;
  char msg[128];

  MMMessage( "Initializing Accelerometer components..." );

  // Read the affine accelerometer map parameters from the main symbol table
  temp = MMGetArraySymbol( "accelerometer_offsets" );
  for ( i = 0; i < temp.getCount(); i++ ) 
    offsets[ i ] = temp.get(i);
  for ( i = temp.getCount(); i < 3; i++ )
    offsets[ i ] = offsetD[i];

  temp = MMGetArraySymbol( "accelerometer_gains" );
  for ( i = 0; i < temp.getCount(); i++ ) 
    gains[ i ] = temp.get(i);
  for ( i = temp.getCount(); i < 3; i++ )
    gains[ i ] = gainD[ i ];


  sprintf( msg, "\n  x:{%1.2f,%1.2f}, y:{%1.2f,%1.2f}, z:{%1.2f,%1.2f}.", 
           offsets[0], gains[0], offsets[1], gains[1], offsets[2], gains[2]);
  MMMessage( msg );

  // Add the PulseWidth modules with the appropriate timer and digital
  // line settings. Note that non-polling modules are used because the
  // RC signals have a sufficiently low frequency.
  pwm[ 0 ] = new PulseWidth( ACCEL_TIMERX, ACCEL_GATEX, 
                             ACCEL_TIMEOUT, true, hw );
  pwm[ 1 ] = new PulseWidth( ACCEL_TIMERY, ACCEL_GATEY, 
                             ACCEL_TIMEOUT, true, hw );
  pwm[ 2 ] = new PulseWidth( ACCEL_TIMERZ, ACCEL_GATEZ, 
                             ACCEL_TIMEOUT, true, hw );
    
  // Add and activate the modules
  for ( i = 0; i < 3; i++ ) {
    MMAddModule ( pwm[ i ], PWM_MODULE_PERIOD, PWM_MODULE_OFFSET, 
                  PWM_MODULE_ORDER );
    MMActivateModule ( pwm[ i ] );
  }

  MMMessage( "done.\n" );

}

// UTAccel::~UTAccel : Class destructor. Removes and deallocates all
// PulseWidth modules
UTAccel::~UTAccel( void ) {

  int count;

  for ( count = 0; count < 3; count++ ) {

    // Remove the PulseWidth modules
    MMRemoveModule ( pwm[ count ] );
    delete pwm[ count ];
  }

}

// UTAccel::read : Reads in the value of a continuous dial.
float UTAccel::read( Axis a ) {
  float val, width;
  float offset, gain;

  // Decide on the axis and retrieve the pulse width and the 
  // calibration parameters 
  if ( a == AXIS_X ) {

    width = pwm[ 0 ]->getWidth();
    offset = offsets[ 0 ];
    gain = gains[ 0 ];

  } else if ( a == AXIS_Y ) {

    width = pwm[ 1 ]->getWidth();
    offset = offsets[ 1 ];
    gain = gains[ 1 ];

  } else if ( a == AXIS_Z ) {

    width = pwm[ 2 ]->getWidth();
    offset = offsets[ 2 ];
    gain = gains[ 2 ];

  }

  // Compute the acceleration
  val = gain * ( width - offset );

  return val;
}

void UTAccel::getInfo( Axis axis, float *resolution, float *limit ) {

  // The resolution is a function of the timer frequency and the 
  // accelerometer range and period.

  // For 2G version
  *resolution = 43.0e-3;
  *limit = 2.0;
}
