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
 * $Id: WalkMachine.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/05/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _WALKMACHINE_HH
#define _WALKMACHINE_HH

#include <stdio.h>
#include <math.h>
#include "ModuleManager.hh"
#include "StateMachine.hh"
#include "PositionControl.hh"
#include "Profiler.hh"

// Module specific constants ---------------------------------------

#define WALK_KP_DFLT 24.015
#define WALK_KD_DFLT 0.67242

#define WALK_TRIPODTIME_DFLT     0.265
#define WALK_ADJTIME_DFLT        0.2
#define WALK_CPGPERIOD_DFLT      0.53
#define WALK_DUTYFACTOR_DFLT     0.6226
#define WALK_SWEEPANGLE_DFLT     0.6981
#define WALK_LEGOFFSET_DFLT      -0.15
#define WALK_SMOOTH_DFLT         0.11
#define WALK_TURNOFFSET_DFLT     0.113
#define WALK_TURNSWEEPANGLE_DFLT 0.0
#define WALK_TURNDUTYFACTOR_DFLT 0.0
#define WALK_CUBIC_DFLT          true

// Type definitions  -----------------------------------------------

// Parameters modulating walking behavior
class WalkParam_t {

public:
  WalkParam_t( void ) { };
  WalkParam_t( WalkParam_t & );

  WalkParam_t& operator=( const WalkParam_t & );

public:

  double tripodTime;      // Time to prepare and bring down the first tripod
  double standAdjTime;    // Time to wait between leg offset adjustments while standing
  double cpgPeriod;       // period of the whole step cycle    
  float dutyFactor[6];   // Ratio of stance time to  ?
  float sweepAngle[6];   // Angle span during the slow leg sweep
  float legOffset[6];    // Angle offset for the whole profile
  MotorGains_t gains[6]; // Current motor gains
  float smooth;          // Ratio for the smoothing speed transitions
  float turnOffset;      // Differential leg offset adjustment for turning 
  float turnSweepAngle;  // Differential sweep angle adjustment for turning
  float turnDutyFactor;  // Differential duty factor adjustment for turning
  bool cubic;            // Flag to choose between linear or cubic spline smoothing

};


// The WalkMachine class -------------------------------------------
class WalkMachine : public StateMachine {

public:

  WalkMachine( void );
  ~WalkMachine( void );

  void init( void );
  void activate( void );
  void deactivate( void );
  void uninit( void );

  void setSpeedCommand( float cmd );
  void setTurnCommand( float cmd );
  
  void setUpside( bool down ) { upsideDownFlag = down; };

  void setParams( WalkParam_t *p ) { nextParams = p; };
  void getParams( WalkParam_t *p ) { memcpy( p, &curParams, sizeof(WalkParam_t) ); };

  bool isDone( void ) const { return done; }

private:

  // events
  EventObject ( TripodReady ) * tripodReady;
  EventObject ( HalfDone ) * halfDone;
  EventObject ( AllDone ) * allDone;
  EventObject ( WalkTrigEv ) * walkTrigEv;
  EventObject ( NoCommand ) * noCommand;
  EventObject ( TripodDown ) * tripodDown;
  EventObject ( StandAdjEv ) * standAdjEv;

  // states
  StateObjectEDX ( WaitingCommand ) * waitingCommand;
  StateObjectEDX ( LiftingTripod  ) * liftingTripod;
  StateObjectEDx ( FirstHalf ) * firstHalf;
  StateObjectEDx ( SecondHalf ) * secondHalf;
  StateObjectEDx ( LoweringTripod ) * loweringTripod;

  // data
  PositionControl * control[6];
  double mark;
  CPG * cpg;
  bool done;
  Profiler prof[6];
  CPGProfiler cpgProf[6];

  // These parameters are read from the config file and set up in the
  // constructor of this class. The default values are in case the config
  // file does not provide the information
  WalkParam_t curParams;    // Parameters currently in use
  WalkParam_t *nextParams;  // Parameters to take effect in the next half cycle
  
  bool upsideDownFlag;
  bool firstHalfFlag;

  float speedCommand;
  float turnCommand;

  float lastSpeedCommand;
  float lastTurnCommand;

  MotorGains_t oldGains[6];     // Old PD motor gains before activation

  void setupProfiles( void );
  void useNextParams( void );
  void gotoAngle( uint leg, double time, float angle, int direction );
  void trackCurrentProfile ( void );
  void trackCPGProfile ( void ) ;

};

#endif







