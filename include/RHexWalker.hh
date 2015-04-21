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
 * $Id: RHexWalker.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Created       : Uluc Saranli, 07/05/2001
 * Last Modified : Uluc Saranli, 07/05/2001
 *
 ********************************************************************/

#ifndef _RHEXWALKER_HH
#define _RHEXWALKER_HH

// Local includes
#include "basicmath.hh"
#include "ModuleManager.hh"
#include "WalkMachine.hh"
#include "ExtProfiler.hh"
#include "SlopeEstimator.hh"

// -----------------------------------------------------------------
// Module specific constants ---------------------------------------

#define TRIWHEEL_TURNSWEEPANGLE_DFLT 0.1
#define TRIWHEEL_TRIPODTIME_DFLT     0.3
#define TRIWHEEL_MAXCPGPERIOD_DFLT   2.0
#define TRIWHEEL_MINCPGPERIOD_DFLT   0.4

// Parameters for the inclination compensation functionality -------- 
typedef struct {

  float  gamma;
  float  iota;
  float  ksi;
  float  epsilon;

} InclineCompParam_t;

// -----------------------------------------------------------------
// The RHexWalker class --------------------------------------------

class RHexWalker : public Module {

public:
  typedef enum { TWO_STROKE_TRIPOD_GAIT, TRIWHEEL_GAIT } WalkingGait;

  RHexWalker( void );

  // Module base class interface
  void init ( void );
  void uninit ( void ) { };
  void activate ( void );
  void deactivate ( void );
  void update ( void );

  // RHexWalker interface ----------------------------------------------------

  // Set current speed in the range [-1,1]  
  void setForwardCommand( float cmd );

  // Set turning command in the range [-1,1]
  void setTurnCommand( float cmd );

  // Enable/disable inclination compensation
  void inclineEnable( bool enable ) { inclineFlag = enable; };

  // Sets the walking gait to be used. Effective after next module activation.
  void setGait( WalkingGait newGait ) { nextGait = newGait; };

  // Manually set the leg offset ( only effective if the incl. comp. is off )
  void setLegOffset( float offset ) { manualLegOffset = offset; }

  // Sets the walker's assumption about robot polarity
  void setUpside( bool down ) { wm->setUpside( down ); };

  // Flag to indicate that walker is ready to exit.
  bool isDone( void ) { return wm->isDone(); };

private:

  // Basic walking state machine and related stuff
  WalkMachine  * wm;
  WalkParam_t    walkParams;

  float speedCommand, turnCommand;
  float manualLegOffset, curLegOffset;
  float walkSpeed;

  // Different gaits
  WalkingGait curGait, nextGait;

  // Variable speed alternating triwheel stuff.
  float triwheelTurnSweepAngle;
  float triwheelTripodTime;
  float triwheelMinCPGPeriod;
  float triwheelMaxCPGPeriod;

  // Variable speed standard walking stuff.
  float  minSpeed, maxSpeed;

  // The Function profile for each variable
  ExtProfiler tripodTimeProf,
              cpgPeriodProf,
              dutyFactorProf,
              sweepAngleProf,
              legOffsetFProf,
              legOffsetBProf,
              smoothProf,
              turnOffsetProf,
              turnDutyFactorProf,
              turnSweepAngleProf;

  // Inclination estimation related stuff
  SlopeEstimator     *slope;
  InclineCompParam_t  inclineParams;
  bool                inclineFlag;

  void computeWalkParams( void );
  void readConfiguration( void );

};

#endif
