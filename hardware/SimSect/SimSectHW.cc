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
 * $Id: SimSectHW.cc,v 1.4 2001/07/18 21:33:08 ulucs Exp $
 *
 * Low level hardware interface to the SimSect hardware components
 *
 * Created       : Uluc Saranli, 03/11/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifdef _LINUX_      // Only compile this file for Linux

// System includes
#include <math.h>
// local SimSect includes
#include "SS_Util.hh"
// local RHexLib includes
#include "StdModules.hh"
#include "SimSectHW.hh"
#include "SimComponents.hh"
#include "VirtualInput.hh"
#include "SimSectDriver.hh"

// Virtual motor simulators for all 6 legs
static VirtualInput *dialInputs;
static VirtualInput *switchInputs;
SimSectDriver *driver;

// Start in the uninitialized state
bool SimSectHW::initStatus = false;

// SimSectHW::initialize: Creates the virtual hardware component interfaces
void SimSectHW::initialize( void ) {

  char filename[128];
  char msg[128];

  if ( !initStatus ) {
    // This is the first time a virtual hardware is created

    MMMessage( "Initializing low level hardware interface ( SimSect )...\n" );

    // Create and add the virtual dials and switches. The order is set to low
    // so that they get updated first
    dialInputs = new VirtualInput( "simsectdials" );

    // Get the filename from the symbol table
    MMGetStringSymbol( "simsect_dials", filename, "dials.scr" );
    sprintf( msg, "Loading dial script %s...\n", filename );
    MMMessage( msg );

    // Initialize and add the dial virtual inputs
    dialInputs->initialize( filename );
    MMAddModule( dialInputs, 1, 0, 1 );
    MMActivateModule( dialInputs );

    switchInputs = new VirtualInput( "simsectswitches" );

    // Get the filename from the symbol table
    MMGetStringSymbol( "simsect_switches", filename, "switches.scr" );
    sprintf( msg, "Loading switch script %s...\n", filename );
    MMMessage( msg );

    // Initialize and add the switch virtual inputs
    switchInputs->initialize( filename );
    MMAddModule( switchInputs, 1, 0, 1 );
    MMActivateModule( switchInputs );

    MMMessage( "Creating the SimSectDriver interface...\n" );
    // Create the SimSect driver module and add it with the appropriate order etc.
    driver = new SimSectDriver;
    MMAddModule( driver, 1, 0, OTHER_MODULES );
    MMActivateModule( driver );
    MMMessage( "done.\n" );
    
    // Create all the hardware components for the current hardware object
    encoders = new SimEncoder;
    analogIO = new SimAnalog;
    digitalIO = new SimDigital;
    timers = new SimTimer;
    accels = new SimAccel;
    gyros = new SimGyro;
    power = new SimPower;
    switches = new SimSwitch;
    dials = new SimDial;
    dcmotors = new SimMotor;

    initStatus = true;

    MMMessage( "Hardware interface successfully initialized ( SimSect ).\n" );

  } else {
    // More than one Virtual hardware object created!
    MMWarning( "SimSectHW::initialize", "Trying to initialize more than once!");
  }
}

// SimSectHW::cleanup : Removes modules etc.
void SimSectHW::cleanup( void ) {

  if ( initStatus ) {

    // Remove and delete the VirtualInput objects
    MMRemoveModule( dialInputs );
    delete dialInputs;
    MMRemoveModule( switchInputs );
    delete switchInputs;
    MMRemoveModule( driver );
    delete driver;

    // Delete the previously allocated hardware component objects 
    if ( encoders != NULL ) delete encoders;
    if ( analogIO != NULL ) delete analogIO;
    if ( digitalIO != NULL ) delete digitalIO;
    if ( timers != NULL ) delete timers;
    if ( accels != NULL ) delete accels;
    if ( gyros != NULL ) delete gyros;
    if ( power != NULL ) delete power;
    if ( switches != NULL ) delete switches;
    if ( dials != NULL ) delete dials;
    if ( dcmotors != NULL ) delete dcmotors;

    initStatus = false;
  } else {
    // This SimSect hardware instance is not the first one created!
  }
}

// SimSectHW::~SimSectHW : Class destructor
SimSectHW::~SimSectHW( ) {

  if ( initStatus ) {
    // Somebody exited without proper cleanup!
    cleanup();
  }
}

CLOCK SimSectHW::readClock( void ) {
  // Currently, we use the Module Manager time steps

  return MMGetStepPeriod() * ( CLOCK( MMGetStepCount() ) + CLOCK( 1 ) );

};

void SimEncoder::enable( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "SimEncoder::read", "Encoder index out of bounds!" );
    return;
  }

  driver->enableEncoder( index, true );
}

void SimEncoder::disable( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "SimEncoder::read", "Encoder index out of bounds!" );
    return;
  }

  driver->enableEncoder( index, false );
}


// Reads the encoder value from the corresponding virtual motor
uint16 SimEncoder::read( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "SimEncoder::read", "Encoder index out of bounds!" );
    return 0;
  }

  return driver->readEncoder( index );

}

// Resets the encoder value for the corresponding virtual motor
void SimEncoder::reset( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "Simencoder::reset", "Encoder index out of bounds!" );
    return;
  }

  driver->resetEncoder( index );

}

// Sets the analog command output value (V) for the corresponding virtual motor.
void SimAnalog::write( uint index, float value) {

  float command;

  if ( index >= 6 ) {
    MMWarning ( "SimAnalog::write", "Analog output index out of bounds!" );
    return;
  }

  // Convert the analog command to motor terminal voltage.
  command 
    = SIMSECT_DRIVE_SUPPLY * SIMSECT_DRIVE_SCALE 
    * ( value - SIMSECT_DRIVE_OFFSET );

  driver->setCommand( index, command );
  
}

// Reads in an analog input channel (V)
// For the time being, assume that all analog inputs return 0.0
float SimAnalog::read( uint index ) {

  if ( index >= 16 )
    MMWarning ( "SimAnalog::read", "Analog input index out of bounds!" );

  return 0;
}

bool SimSwitch::read( uint index ) {

  // Read the virtual inputs for the switches and decide on the
  // switch configuration with a threshold on the value
  if ( switchInputs->getChannelValue( index ) > 0.5 )
    return true;
  else 
    return false;

}

float SimDial::read( uint index ) {

  float value = dialInputs->getChannelValue( index );

  // Clip the value to dial range

  if ( value > 1.0 ) value = 1.0;
  if ( value < -1.0 ) value = -1.0;

  return value;
}

//
// SimAccel::read : Reads in the value of an accelerometer axis.
//
float SimAccel::read( Axis a ) {
  float val;
  double vect[3], gravity[3], R[9], Rinv[9], accel[3];

  // Transform the gravity vector into the body coordinates
  vect[0] = 0.0; vect[1] = 0.0; vect[2] = -9.81;
  driver->bodyRotation( R );
  inverse_matrix( Rinv, R );
  mult_mat_vect( gravity, Rinv, vect );

  // Determine the accelerometer output as the sum of the gravity offset and
  // the actual acceleration
  driver->bodyAcceleration( vect );
  add_vect_vect( accel, vect, gravity );

  if ( a == AXIS_X ) {

    val = accel[0];

  } else if ( a == AXIS_Y ) {

    val = accel[1];

  } else if ( a == AXIS_Z ) {

    val = accel[2];

  }

  return val;
}

void SimAccel::getInfo( Axis axis, float *resolution, float *limit ) {

  // The resolution is a function of the timer frequency and the 
  // accelerometer range and period.

  // For 2G version
  *resolution = 1e-9;
  *limit = 10.0;
}


// SimGyro::readRate 
//
// The rotation rate of the body around a particular axis is the projection of 
// the angular velocity vector in the body frame on that axis.
//
float SimGyro::readRate( Axis a ) {

  float val;
  double angularvel[3];

  // Retrieve the angular velocity in the body frame
  driver->bodyAngularVel( angularvel );

  if ( a == AXIS_X ) {

    val = angularvel[0];

  } else if ( a == AXIS_Y ) {

    val = angularvel[1];

  } else if ( a == AXIS_Z ) {

    val = angularvel[2];

  }

  return val;
}

//
// SimGyro::readAngle : Reads in the value of gyro axis.
//
float SimGyro::readAngle( Axis a ) {

  float val;
  double R[9];

  // Retrieve the body rotation matrix
  driver->bodyRotation( R );

  // Determine the accelerometer output as the sum of the gravity offset and
  // the actual acceleration

  if ( a == AXIS_X ) {

      val = atan2(R[3],R[0]);

  } else if ( a == AXIS_Y ) {

      val = atan2(-R[6],sqrt(R[7]*R[7]+R[8]*R[8]));

  } else if ( a == AXIS_Z ) {

      val = atan2(R[7], R[8]);

  }

  return val;
}

void SimGyro::getInfo( Axis axis, float *resolution, float *limit ) {

  // The resolution is a function of the timer frequency and the 
  // accelerometer range and period.

  // For 2G version
  *resolution = 1e-9;
  *limit = 10.0;
}

#endif
