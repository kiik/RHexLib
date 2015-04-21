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

/** @file UTDial.cpp
 *  @brief Low level hardware interface to the continuous dial facilities of the University of Tartu version of RHex.
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

static PulseWidth *pwm[6];

// UTDial::UTDial : Class constructor.
// Creates and initializes the PulseWidth modules.
UTDial::UTDial( Hardware *hw ) {

  int count;

  MMMessage( "Initializing Dial components..." );

  // Add the PulseWidth modules with the appropriate timer and digital line
  // settings. Note that non-polling modules are used because the RC signals
  // have a sufficiently low frequency.
  pwm[ 0 ] = new PulseWidth( RC_TIMER0, RC_GATE0, RC_TIMEOUT, false, hw );
  pwm[ 1 ] = new PulseWidth( RC_TIMER1, RC_GATE1, RC_TIMEOUT, false, hw );
  pwm[ 2 ] = new PulseWidth( RC_TIMER2, RC_GATE2, RC_TIMEOUT, false, hw );
  pwm[ 3 ] = new PulseWidth( RC_TIMER3, RC_GATE3, RC_TIMEOUT, false, hw );
  pwm[ 4 ] = new PulseWidth( RC_TIMER4, RC_GATE4, RC_TIMEOUT, false, hw );
  pwm[ 5 ] = new PulseWidth( RC_TIMER5, RC_GATE5, RC_TIMEOUT, false, hw );
    
  for ( count = 0; count < 6; count++ ) {

    // Add and activate the modules
    MMAddModule ( pwm[count], PWM_MODULE_PERIOD, PWM_MODULE_OFFSET, PWM_MODULE_ORDER );

    MMActivateModule ( pwm[ count ] );
  }

  MMMessage( "done.\n" );

}

// UTDial::~UTDial : Class destructor. Removes and deallocates all
// PulseWidth modules
UTDial::~UTDial( void ) {

  int count;

  for ( count = 0; count < 6; count++ ) {

    // Remove the PulseWidth modules
    MMRemoveModule ( pwm[ count ] );
    delete pwm[ count ];
  }

}

// UTDial::read : Reads in the value of a continuous dial
float UTDial::read( uint index ) {
  float val, width;
  
  width = pwm[ index ]->getWidth();
 
  // If the pulse width reading is outside the range, return 0.0
  if ( width > RC_MAX_WIDTH || width < RC_MIN_WIDTH )
    return 0.0;

  // Map the pulse width reading to the range [ -1.0, 1.0 ]
  val = 2.0 * ( pwm[ index ]->getWidth() - RC_MID_WIDTH ) / RC_RANGE;

  // Clip the PWM reading to the interval
  if ( val < -1.0 ) val = -1.0;
  if ( val > 1.0 ) val = 1.0;

  // Switch the polarities of the first and third channels. This is
  // done because the forward backward levers on the RC normally
  // result in negative values when pushed forward, which is not what
  // we want.
  if ( index == 0 || index == 2 )
    val = -val;

  return val;
}

