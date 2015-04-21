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
 * $Id: RHexWalker.cc,v 1.4 2001/08/08 22:14:59 usofrob Exp $
 *
 * Created       : Uluc Saranli, 07/05/2001
 * Last Modified : Uluc Saranli, 07/17/2001
 *
 ********************************************************************/

#include "RHexWalker.hh"
#include "StdModules.hh"
#include "StandMachine.hh"

RHexWalker::RHexWalker( void ) 
  : Module( RHEXWALKER_NAME, 0, true, false ) {

  // Initialize fields
  wm = NULL;
  slope = NULL;

  speedCommand = turnCommand = 0.0;
  manualLegOffset = 0;

  walkSpeed = minSpeed = maxSpeed = 1.0;

  // Default gait
  curGait = nextGait = TWO_STROKE_TRIPOD_GAIT;

  // Inclination compensation
  inclineFlag = false;   // Default to no compensation
  
}

void RHexWalker::init( void ) {

  // Find the modules that we will use.
  if ( ( wm = ( WalkMachine * ) 
         MMFindModule( WALKMACHINE_NAME, 0 )) == NULL)
    MMFatalError ( "RHexWalker::init", "Cannot find the walking controller!" );

  if ( ( slope = ( SlopeEstimator * ) 
         MMFindModule( SLOPEESTIMATOR_NAME, 0 )) == NULL)
    MMFatalError ( "RHexWalker::init", "Cannot find the slope estimator!" );

  // Read the configuration symbols and configure the piecewise linear
  // parameter interpolation settings.
  readConfiguration();
}


void RHexWalker::activate( void ) {

  // Start using the last gait setup
  curGait = nextGait;

  // Reset the commands
  setForwardCommand( 0.0 );
  setTurnCommand( 0.0 );

  // Activate the WalkMachine and the SlopeEstimator
  MMGrabModule( wm, this );

  if ( curGait == TWO_STROKE_TRIPOD_GAIT )
    MMGrabModule( slope, this );
}

void RHexWalker::deactivate( void ) {

  // Deactivate the WalkMachine and the SlopeEstimator
  MMReleaseModule( wm, this );

  if ( curGait == TWO_STROKE_TRIPOD_GAIT )
    MMReleaseModule( slope, this );

}

void RHexWalker::update( void ) {

  int   i;
  float temp;

  if ( curGait == TWO_STROKE_TRIPOD_GAIT ) {
    // The normal, alternating tripod controller is active.

    if ( inclineFlag ) {
      // Automatic leg offset adjustment turned on.
      // Select the leg offset based on inclination compensation

      float gamma, iota, ksi, epsilon;

      float alpha, alpha_new;
      float sign_alpha;
      float sign_speed;

      float alpha1, alpha2, alpha3;
      float beta1, beta2, beta3;

      float siota;

      float l = 0.17, h = 0.205;

      alpha = slope->readSlope();

      // ---- Set the parameters for the compensation
      gamma = inclineParams.gamma;
      iota = inclineParams.iota;
      ksi = inclineParams.ksi;
      epsilon = inclineParams.epsilon;

      sign_speed = 1.0;

      alpha_new  = sign_speed * alpha;
      siota      = sin( fabs ( iota * alpha_new ) );
      sign_alpha = sign( alpha_new );

    // forward leg
      beta1  = M_PI / 2 - ( gamma * alpha_new );
      alpha1 = M_PI / 2 - beta1 + ( iota * alpha_new );
      // middle leg
      beta2 = ( asin( sin( beta1 ) - h/l * siota ) );
      alpha2 = sign_alpha * ( M_PI / 2 - beta2 + ( iota * alpha_new ) );
      // Rear legs
      beta3  = ( asin( sin( beta1 ) - 2*h/l * siota) );
      alpha3 = sign_alpha * ( M_PI / 2 - beta3 + ( iota * alpha_new ) );

    // ---- Leg speed profile sweep angle adjustment
      temp = sweepAngleProf.getVal( walkSpeed )
        + saturate( ksi * alpha_new, 0, 0.5 );
      for (i = 0 ; i < 6 ; i++)
        walkParams.sweepAngle[i] = temp;

    // ---- Leg speed profile duty cycle adjustment
      temp = dutyFactorProf.getVal( walkSpeed )
        + saturate( epsilon * alpha_new, 0, 0.2 );
      for (i = 0 ; i < 6 ; i++)
        walkParams.dutyFactor[i] = temp;

      walkParams.legOffset[0] = sign_speed * curLegOffset - gamma * alpha3;
      walkParams.legOffset[1] = sign_speed * curLegOffset - gamma * alpha2;
      walkParams.legOffset[2] = sign_speed * curLegOffset - gamma * alpha1;
      walkParams.legOffset[3] = sign_speed * curLegOffset - gamma * alpha3;
      walkParams.legOffset[4] = sign_speed * curLegOffset - gamma * alpha2;
      walkParams.legOffset[5] = sign_speed * curLegOffset - gamma * alpha1;

    } else {
      // Automatic leg offset adjustment turned off.
      // Choose the leg offset based on the external manual setting.
    }
  } else {
    // The alternating triwheel controller is active.
    // Currently, there are no parameter modulations for this controller.
    for ( i = 0; i < 6; i++ )
      walkParams.legOffset[i] = 0.0;
  }

  // Inform the WalkMachine about the parameter changes.
  wm->setParams( &walkParams );
}

// RHexWalker::computeWalkParams
//
// Ggiven the current speed and turning commands in the range [-1,1],
// this function computes the parameters for the WalkMachine depending
// on the current gait type and the configuration settings. Note that
// further changes to the variables are also made by the inclination
// compensation etc.
void RHexWalker::computeWalkParams( void ) {

  float temp1, temp2;
  float validCommand;
  double temp3;
  int i;

  // If we are running forward, speedCommand is uses as a basis for
  // the speed setting. If we are turning in place, turnCommand is
  // used, because speedCommand will be 0.0.
  if ( speedCommand != 0.0 )
    validCommand = speedCommand;
  else
    validCommand = turnCommand;

  if ( curGait == TWO_STROKE_TRIPOD_GAIT ) {
    // The normal, alternating tripod controller is active.  Choose
    // the walking parameters based on the current speed setting and
    // the configuration from the file.

    // Compute the walk speed setting value using the current command
    walkSpeed = minSpeed + fabs( validCommand ) * (maxSpeed - minSpeed);

    // Use Function Profiles to set Parameters
    walkParams.cpgPeriod = cpgPeriodProf.getVal( walkSpeed );
    walkParams.turnOffset = turnOffsetProf.getVal( walkSpeed );
    walkParams.turnSweepAngle = turnSweepAngleProf.getVal( walkSpeed );
    walkParams.turnDutyFactor = turnDutyFactorProf.getVal( walkSpeed );
    walkParams.smooth = smoothProf.getVal( walkSpeed );

    temp1 = dutyFactorProf.getVal( walkSpeed );
    temp2 = sweepAngleProf.getVal( walkSpeed );
    temp3 = tripodTimeProf.getVal( walkSpeed );

    for( i = 0; i < 6; i++ ) {
      walkParams.dutyFactor[i] = temp1;
      walkParams.sweepAngle[i] = temp2;
      walkParams.tripodTime = temp3;
    }

    if ( inclineFlag ) {
      // Inclination compensation is turned on!

      // Determine whether forw or back leg offset parameters should be used.
      if ( speedCommand < 0.0 ) {
        // Speed command is negative. Use backward offsets

        curLegOffset = legOffsetBProf.getVal( walkSpeed );

      } else {
        // Speed command is positive. Use normal offsets

        curLegOffset = legOffsetFProf.getVal( walkSpeed );
      }
    } else
      // If the inclination compensation is off, use the manual offset
      // set through the method setLegOffset().
      curLegOffset = manualLegOffset;

  } else {
    // The alternating triwheel controller is active.  The walking
    // parameter settings are much more constrained. Specifically,
    // duty_factor = 0.5, sweep_angle = PI and leg_offset = 0. Only
    // the cpg_period modulates speed.

    // Set the speed independent parameters
    walkParams.turnSweepAngle = triwheelTurnSweepAngle;
    walkParams.turnOffset = 0.0;
    walkParams.turnDutyFactor = 0.0;
    // Smooth is only effective for differential turning. The profiles
    // are already constant speed for triwheel walking
    walkParams.smooth = 0.1;

    for( i = 0; i < 6; i++ ) {
      walkParams.dutyFactor[i] = 0.5;
      walkParams.sweepAngle[i] = M_PI;
      walkParams.legOffset[i] = 0.0;
      walkParams.tripodTime = triwheelTripodTime;
    }

    // Determine the CPG period based on the minimum allowed CPG
    // period and the current speed setting. Note that speed is
    // inversely related to the CPG period
    temp1 = 1.0 / triwheelMaxCPGPeriod 
      + fabs( validCommand ) 
      * ( 1.0 / triwheelMinCPGPeriod - 1.0 / triwheelMaxCPGPeriod );

    walkParams.cpgPeriod  = 1.0 / temp1;
  }

  // Inform the WalkMachine about the parameter changes.
  wm->setParams( &walkParams );
}

void RHexWalker::setForwardCommand( float cmd ) {

  speedCommand = cmd;

  computeWalkParams();

  // Set the WalkMachine's speed command
  if ( speedCommand < 0 )
    wm->setSpeedCommand( -1.0 );
  else if ( speedCommand > 0 )
    wm->setSpeedCommand( 1.0 );
  else 
    wm->setSpeedCommand( 0.0 );

}

void RHexWalker::setTurnCommand( float cmd ) {

  turnCommand = cmd;

  computeWalkParams();

  // Set the WalkMachine's turn command
  if ( turnCommand < 0 )
    wm->setTurnCommand( -1.0 );
  else if ( turnCommand > 0 )
    wm->setTurnCommand( 1.0 );
  else 
    wm->setTurnCommand( 0.0 );

}

void RHexWalker::readConfiguration( void ) {

  Floats temp, walkSpeeds;
  int i;
  bool errorFlag = false;

  walkSpeeds = MMGetArraySymbol( "walk_speeds" ); // Read in speed values 

  // If it's not defined, then use default values
  if ( walkSpeeds.getCount() == 0.0 ) {

    // Check to see if any of the configuration symbols are defined.
    // This is not allowed if walk_speeds is not defined
    temp = MMGetArraySymbol( "walk_cpg_period" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_duty_factor" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_sweep_angle" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_leg_offset_forward" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_leg_offset_backward" );
    if ( temp.getCount() > 0 )  errorFlag = true;
    temp = MMGetArraySymbol( "walk_smooth_factor" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_turn_offset" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_turn_sweep_angle" );
    if ( temp.getCount() > 0 ) errorFlag = true;
    temp = MMGetArraySymbol( "walk_turn_duty_factor" );
    if ( temp.getCount() > 0 ) errorFlag = true;

    if ( errorFlag )
      MMFatalError( "RHexWalker::readWalkingParams", 
                    "Must define Walk speed to define cpgPeriod." );


    MMWarning( "RHexWalker::readWalkingParams", 
               "walk_speeds undefined, using default parameters!" );

    // When the walk_speeds is not defined, we only define a single speed
    // and configure the parameters settings accordingly.
    cpgPeriodProf.addSegment( 1.0, WALK_CPGPERIOD_DFLT );
    dutyFactorProf.addSegment( 1.0, WALK_DUTYFACTOR_DFLT );
	tripodTimeProf.addSegment( 1.0, WALK_TRIPODTIME_DFLT );
    sweepAngleProf.addSegment( 1.0, WALK_SWEEPANGLE_DFLT );
    legOffsetFProf.addSegment( 1.0, WALK_LEGOFFSET_DFLT );
    legOffsetBProf.addSegment( 1.0, WALK_LEGOFFSET_DFLT );
    smoothProf.addSegment( 1.0, WALK_SMOOTH_DFLT );
    turnOffsetProf.addSegment( 1.0, WALK_TURNOFFSET_DFLT );
    turnDutyFactorProf.addSegment( 1.0, WALK_TURNDUTYFACTOR_DFLT );
    turnSweepAngleProf.addSegment( 1.0, WALK_TURNSWEEPANGLE_DFLT );

    minSpeed = maxSpeed = 1.0;

  } else {

    // cpg period 
    temp = MMGetArraySymbol( "walk_cpg_period" );
    if ( walkSpeeds.getCount() == temp.getCount() )     {
      // If they are the same length then setup the profile

      cpgPeriodProf.setup( walkSpeeds, temp );

    } else
      // The profile can't be created, so error out
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_cpg_period have different # of elements." );

    // tripod time
    temp = MMGetArraySymbol( "walk_tripod_time" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      tripodTimeProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_tripod_time have different # of elements.." );

    // duty factor
    temp = MMGetArraySymbol( "walk_duty_factor" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      dutyFactorProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_duty_factor have different # of elements.." );

    // sweep angle
    temp = MMGetArraySymbol( "walk_sweep_angle" );
    if (walkSpeeds.getCount() == temp.getCount() ) {

      sweepAngleProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_sweep_angle have different # of elements." );

    // forward running offset
    temp = MMGetArraySymbol( "walk_leg_offset_forward" );
    if (walkSpeeds.getCount() == temp.getCount() ) {

      legOffsetFProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_leg_offset_forward have different # of elements." );

    // backward running offset
    temp = MMGetArraySymbol( "walk_leg_offset_backward" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      legOffsetBProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_leg_offset_backward have different # of elements." );

    // smoothing factor
    temp = MMGetArraySymbol( "walk_smooth_factor" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      smoothProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_smooth_factor have different # of elements." );

    // turning offset
    temp = MMGetArraySymbol( "walk_turn_offset" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      turnOffsetProf.setup( walkSpeeds, temp );

    } else
      MMFatalError("RHexWalker::readConfiguration", "walk_speeds and walk_turn_offset have different # of elements." );

    // turning angle offset
    temp = MMGetArraySymbol( "walk_turn_sweep_angle" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      turnSweepAngleProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_turn_sweep_angle angle have different # of elements." );


    // turning "duty factor" offset
    temp = MMGetArraySymbol( "walk_turn_duty_factor" );
    if ( walkSpeeds.getCount() == temp.getCount() ) {

      turnDutyFactorProf.setup( walkSpeeds, temp );

    } else
      MMFatalError( "RHexWalker::readConfiguration", "walk_speeds and walk_turn_duty_factor have different # of elements." );

    // walking KP gains
    temp = MMGetArraySymbol( "walk_kp" );
    for ( i = 0; i < temp.getCount(); i++ ) 
      walkParams.gains[i].kp = temp.get(i);

    for ( i = temp.getCount(); i < 6; i++ ) {
      walkParams.gains[i].kp = WALK_KP_DFLT;
      MMWarning( "RHexWalker::readConfiguration", 
                 "Using default parameters for walk_kp!" );
    }

    // walking KD gains
    temp = MMGetArraySymbol( "walk_kd" );
    for ( i = 0; i < temp.getCount(); i++ ) 
      walkParams.gains[i].kd = temp.get(i);

    for ( i = temp.getCount(); i < 6; i++ ) {
      walkParams.gains[i].kd = WALK_KD_DFLT;
      MMWarning( "RHexWalker::readConfiguration", 
                 "Using default parameters for walk_kd!" );
    }


    // Set the minimum and maximum speeds
    minSpeed = walkSpeeds.get(0);
    maxSpeed = walkSpeeds.get(walkSpeeds.getCount() - 1);

  }

  walkSpeed = minSpeed;

  walkParams.cubic = bool( MMGetFloatSymbol( "walk_cubic", WALK_CUBIC_DFLT ) );
  walkParams.standAdjTime 
    = MMGetFloatSymbol( "stand_adj_time", STAND_ADJTIME_DFLT );

  inclineParams.gamma = MMGetFloatSymbol( "walk_forward_gamma", 0.55 );
  inclineParams.iota = MMGetFloatSymbol( "walk_forward_iota", 0.01 );
  inclineParams.ksi = MMGetFloatSymbol( "walk_forward_ksi", 0.3 );
  inclineParams.epsilon = MMGetFloatSymbol( "walk_forward_epsilon", 0.045 );

  // Alternating triwheel controller parameters
  triwheelTurnSweepAngle 
    = MMGetFloatSymbol( "triwheel_turn_sweep_angle",
                        TRIWHEEL_TURNSWEEPANGLE_DFLT );
  triwheelTripodTime 
    = MMGetFloatSymbol( "triwheel_tripod_time",
                        TRIWHEEL_TRIPODTIME_DFLT );
  triwheelMinCPGPeriod 
    = MMGetFloatSymbol( "triwheel_min_cpg_period",
                        TRIWHEEL_MINCPGPERIOD_DFLT );;
  triwheelMaxCPGPeriod 
    = MMGetFloatSymbol( "triwheel_max_cpg_period",
                        TRIWHEEL_MAXCPGPERIOD_DFLT );;

  
}

