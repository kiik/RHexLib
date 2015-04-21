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
 * $Id: StandMachine.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/04/2001
 * Last Modified : Eric Klavins, 01/04/2001
 *
 ********************************************************************/

#ifndef _STANDMACHINE_HH
#define _STANDMACHINE_HH

#include <stdio.h>
#include <math.h>
#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "PositionControl.hh"
#include "EncoderReader.hh"
#include "Profiler.hh"

// Module specific constants ---------------------------------------

#define STAND_KP_DFLT 24.015
#define STAND_KD_DFLT 0.67242

#define STAND_ONETIME_DFLT 0.8
#define STAND_TWOTIME_DFLT 0.4
#define STAND_ADJTIME_DFLT 0.2
#define STAND_LEGOFFSET_DFLT 0.0

// Type definitions  -----------------------------------------------

// The StandMachine class ------------------------------------------
class StandMachine : public StateMachine {

public:

  StandMachine ( void );
  ~StandMachine ( void );
  void init ( void );
  void activate ( void );
  void deactivate ( void );

  void setLegOffset( float offset ) { curLegOffset = offset; };
  bool isDone ( void ) { return done; };

private:

  // events
  EventObject ( StandOneEv ) * standOneEv;  // legs at int, standing position?
  EventObject ( StandTwoEv ) * standTwoEv;  // legs at done standing position?
  EventObject ( OffsetAdjEv ) * offsetAdjEv;// Time to adjust leg offset value

  // states
  StateObjectEDx ( StandOne ) * standOne;   // going to int. standing position
  StateObjectEDx ( StandTwo ) * standTwo;   // going to final standing position
  StateObjectEDx ( StandDone ) * standDone; // finished standing

  // Modules used by the StandMachine
  PositionControl * control[6];
  EncoderReader * enc[6];
  Profiler prof[6];

  // Standing related parameters
  MotorGains_t gains[6];
  double       timeOne, timeTwo, offsetAdjTime;
  float        curLegOffset;

  // Dynamic data
  double mark;
  bool done;

  // Old state
  MotorGains_t oldGains[6];

  void gotoAngle( uint leg, double time, float angle );
  void trackCurrentProfile ( void );
};

#endif
