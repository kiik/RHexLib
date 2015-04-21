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
 * $Id: SimSectDriver.cc,v 1.3 2001/07/13 19:49:56 ulucs Exp $
 *
 * The SimSect engine driver
 *
 * This file implements the member functions defined in
 * SimSectDriver.hh
 *
 * Created       : Uluc Saranli, 03/11/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifdef _LINUX_      // Only compile this file for Linux

// System includes
#include <stdio.h>
#include <math.h>
// SimSect related includes
#include "SS_Integrator.hh"
#include "SS_Util.hh"
#include "SS_Hexapod.hh"
#include "SS_HexVoltage.hh"
// Local includes
#include "SimComponents.hh"
#include "SimSectDriver.hh"

//
// SimSectDriver::SimSectDriver : Class constructor
//
SimSectDriver::SimSectDriver( void )
  : Module( "simsectdriver", 0, true, false ) {

  int i;

  // Open a log file and configure SimSect to dump output to the file
  if ( ( logFile = fopen( "SimSect.log", "w" ) ) == NULL )
    MMFatalError( "SimSectDriver::SimSectDriver", " Cannot open SimSect log file!" );
  // SS_SetLogFile( logFile );

  // Create the hexapod system model object
  hexapod = new SS_Hexapod;

  // Create a hip voltage controller object for the hexapod
  controller = new SS_HexVoltage( hexapod );
  hexapod->setController( controller, VOLTAGE_CONTROL );

  // Create the SimSect Integrator object ( SimSect logging is disabled )
  engine = new SS_Integrator( hexapod, "SimSect.rc", false );
  if ( SS_GetError() < 0 )
    MMFatalError( "SimSectDriver::SimSectDriver", "Could not create SimSect interface!" );

  backEMFcoeff = ( 1.0 / SIMSECT_SPEED_CONST ) / SIMSECT_GEAR_RATIO;

  time = 0.0;

  // Initialize internal data
  acceleration[0] = acceleration[1] = acceleration[2] = 0.0;

  for ( i = 0; i < 6; i++ ) {
    speed[i] = 0;
    encCount[i] = 0;

    command[i] = 0.0;

    // Start out with the encoders disabled
    encoderEnabled[i] = false;

    // Assuming that syncStates() for the hexapod got called during
    // the integrator initialization, retrieve the initial hip angle
    // configuration.
    basepos[i] = pos[i] = lastHipAngle[i] = hexapod->getHipAngle( i );
  }
  
  // Check whether something went wrong or not.
  if ( SS_GetError() < 0 )
    MMFatalError( "SimSectDriver::SimSectDriver", " Cannot create SimSect objects!" );
}

// 
// SimSectDriver::SimSectDriver : Class destructor 
//
SimSectDriver::~SimSectDriver( void ) {

  // TODO: ulucs: There is a weird bug which hangs here
  if ( engine != NULL ) delete engine; engine = NULL;

  // Delete previously allocated objects
  if ( hexapod != NULL ) delete hexapod; hexapod = NULL;
  if ( controller != NULL ) delete controller;

  // Terminate SimSect logging to the file.
  fclose( logFile );
}

void SimSectDriver::deactivate ( void ) {
  //  engine->dumpAuxiliary( "SimSect.data" );
};

//
// SimSectDriver::update : Module update
//
void SimSectDriver::update( void ) {

  double curPos, posDiff, twoPi = 2.0 * M_PI;
  int    encDiff, i;
  double Rinv[9];

  // Perform the integration through SimSect
  time += CLOCK_TO_SEC( MMGetStepPeriod() );
  engine->run( time );
  if ( SS_GetError() < 0 )
    MMFatalError( "SimSectDriver::update", "SimSect integration failed!" );

  // Transform the acceleration to the body frame
  inverse_matrix( Rinv, hexapod->getBodyRotation() );
  mult_mat_vect( acceleration, Rinv, hexapod->getBodyAcceleration() );
  mult_mat_vect( angularvel, Rinv, hexapod->getBodyAngularVel() );

  // Compute the encoder counts for all motors
  for ( i = 0; i < 6; i++ ) {
    curPos = hexapod->getHipAngle( i );
    posDiff = curPos - lastHipAngle[i];
    lastHipAngle[i] = curPos;

    // The following computation assumes that the leg speed is small
    // enough that the leg does not go through more than half a
    // rotation in 0.001s. Especially after the gearhead reduction,
    // this always holds true.
    if ( posDiff > M_PI ) posDiff -= twoPi;
    else if ( posDiff < -M_PI ) posDiff += twoPi;

    // Update the current absolute position of the motor shaft.
    pos[i] += posDiff;

    if ( encoderEnabled[i] ) {
      // Convert the shaft pos. in rads to encoder counts
      encDiff = int( ( pos[i] - basepos[i] ) / twoPi
                     * SIMSECT_ENC_RATIO / SIMSECT_GEAR_RATIO ) ;
      // Take the above count MOD 0xffff to get the actual encoder count
      encCount[i] = encDiff - 0xffff * ( encDiff / 0xffff );
    }
  }
}

//
// SimSectDriver::setCommand : Sets the drive command of an axis.
//
void SimSectDriver::setCommand( uint index, double cmd ) {

  // Clip the voltage to the appropriate range
  command[ index ] = fmax( fmin( SIMSECT_DRIVE_SUPPLY, cmd ), -SIMSECT_DRIVE_SUPPLY );

  // Inform the SimSect hexapod voltage controller about the new command
  controller->setVoltage( index, command[ index ] );

};

#endif
