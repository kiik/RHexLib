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
 * $Id: PulseWidth.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The PulseWidth Module. Measures PWM signals through timers.
 *
 * This file implements the functions defined in PulseWidth.hh
 *
 * Created       : Uluc Saranli, 12/27/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "StdModules.hh"
#include "Hardware.hh"
#include "PulseWidth.hh"

// PulseWidth::PulseWidth : Class constructor (automatic finding of
// hardware object )
PulseWidth::PulseWidth( uint timer, uint digital, double to, bool polling ) 
 : Module( PULSEWIDTH_NAME, timer, false, polling ) {

  Hardware *hw = MMGetHardware();

  if ( hw->timers == NULL || hw->digitalIO == NULL )
    MMFatalError ( "PulseWidth::PulseWidth", "Invalid hardware object!" );

  timers = hw->timers;
  digitalIO = hw->digitalIO;

  timerId = timer;
  byteId = digital >> 3;
  bitId = digital & 0x07;
  timeout = to;

}

// PulseWidth::PulseWidth : Class constructor (hardware object supplied)
PulseWidth::PulseWidth( uint timer, uint digital, double to, 
                        bool polling, Hardware *hw ) 
 : Module( PULSEWIDTH_NAME, timer, false, polling ) {

  if ( hw->timers == NULL || hw->digitalIO == NULL )
    MMFatalError ( "PulseWidth::PulseWidth", "Invalid hardware object!" );

  timers = hw->timers;
  digitalIO = hw->digitalIO;

  timerId = timer;
  byteId = digital >> 3;
  bitId = digital & 0x07;
  timeout = to;

}

// PulseWidth::init : Starts the timer and initializes fields.
void PulseWidth::init( void ) {

  pulseWidth = 0.0;
  timestamp = 0;
  lastGate = 0;

  // Configure the associated timer for mode 2, counting down from 0xffff
  timers->setup( timerId, 2, 0 );

  // Read the new timer information
  timers->getInfo( timerId, &timerInitial, &timerFreq );
}

// PulseWidth::update : Updates the pulse width measurement.
void PulseWidth::update( void ) {

  bool gate;
  uint pulse;
  double curTime = MMReadTime();

  gate = digitalIO->getBit( byteId, bitId );

  if ( !gate ) {
    // Gate is 0: The count has completed. Acquire the PWM width

    if ( curTime < lastGate + timeout ) {
      // Recently observed gate = 1. Read the PWM count
      pulse = 0xffff - timers->read( timerId );

      // If the pulse was too short, ignore it to avoid reading
      // spurious spikes
      if ( pulse > 100 ) {
        pulseWidth = float( pulse ) / timerFreq;
        timestamp = curTime;
      }
    } else {
      // Oops, PWM signal timed out. Gate remained 0 too long. Set the
      // value to 0
      pulseWidth = 0;
      timestamp = curTime;
    }

  } else {
    // Gate input is 1. Looks like there is an incoming PWM signal.
    // Record the current time to avoid timeout.
    lastGate = curTime;
  }

}
