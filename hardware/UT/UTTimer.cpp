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

/** @file UTTimer.cpp
 *  @brief ow level hardware interface to the Timer facilities the Tartu University version of RHex
 *
 *  @author Meelik Kiik
 *  @version 0.1
 *  @date 21/04/2015 16:00
 *
 *  Created at 21/04/2015 16:00
 */

#include <math.h>
#include "config.h"
#include "TartuUniHW.h"
#include "UTComponents.h"
#include "io/dm6814.hh"
#include "io/mpc550.hh"

// UTTimer::UTTimer : Class constructor
UTTimer::UTTimer( void ) {
  int timer;

  MMMessage( "Initializing Timer components..." );

  // Setup all the timers to count down once and stop in mode 0
  for ( timer = 0; timer < 9; timer++ ) {
    setup( timer, 0, 1000);
  }

  MMMessage( "done.\n" );
}

// UTTimer::UTTimer : Class destructor
UTTimer::~UTTimer( ) {
  int timer;

  // Setup all the timers to count down once and stop in mode 0
  for ( timer = 0; timer < 9; timer++ ) {
    setup( timer, 0, 1000 );
  }
}

// UTTimer::setup : Sets the mode and frequency of a particular timer
void UTTimer::setup( uint timer, uint mode, uint32 hertz ) {

  switch ( timer ) {

  case 0:
  case 1:
  case 2:
    DM6814Card[0]->setupTimer( timer, mode, hertz );
    break;

  case 3:
  case 4:
  case 5:
    DM6814Card[1]->setupTimer( timer - 3, mode, hertz );
    break;

  case 6:
  case 7:
  case 8:
    MPC550Card->setupTimer( timer - 6, mode, hertz );
    break;

  default:
    // Warning!: Invalid timer id. SHould not happen!
    break;
  }

}

void UTTimer::getInfo( uint timer, uint *initial, uint *clockFreq ) {
  switch ( timer ) {

  case 0:
  case 1:
  case 2:
    DM6814Card[0]->timerInfo( timer, initial, clockFreq );
    break;

  case 3:
  case 4:
  case 5:
    DM6814Card[1]->timerInfo( timer - 3, initial, clockFreq );
    break;

  case 6:
  case 7:
  case 8:
    MPC550Card->timerInfo( timer - 6, initial, clockFreq );
    break;

  default:
    // Warning!: Invalid timer id. SHould not happen!
    *initial = *clockFreq = 0;
    return;
  }

};
