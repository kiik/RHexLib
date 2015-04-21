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
 * $Id: VirtualHW.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Low level hardware interface to the virtual hardware components
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <math.h>
#include "StdModules.hh"
#include "VirtualHW.hh"
#include "VComponents.hh"
#include "VirtualMotor.hh"
#include "VirtualInput.hh"

// Virtual motor simulators for all 6 legs
static VirtualMotor *motors[6];
static VirtualInput *dialInputs;
static VirtualInput *switchInputs;

// Start in the uninitialized state
bool VirtualHW::initStatus = false;

// VirtualHW::initialize: Creates and initializes the virtual motors
// and creates the virtual hardware component interfaces
void VirtualHW::initialize( void ) {
  int count;
  char filename[128];
  char msg[128];

  if ( !initStatus ) {
    // This is the first time a virtual hardware is created

    MMMessage( "Initializing low level hardware interface ( virtual )...\n" );

    // Add the virtual motors
    for ( count = 0; count < 6; count++ ) {
      motors[count] = new VirtualMotor( count );

      // Set the order to a large value to simulate the motors last
      //MMAddModule( motors[count],
      //             MM_STEP( VM_UPDATE_TIME / CLOCK_TO_DOUBLE(MMGetStepPeriod()) ),
      //             0, OTHER_MODULES );
      //MMActivateModule ( motors[count] );
    }

    // Create and add the virtual dials and switches. The order is set to low
    // so that they get updated first
    dialInputs = new VirtualInput( "virtualdials" );
    MMGetStringSymbol( "virtual_dials", filename, "dials.scr" );
    sprintf( msg, "Loading dial script %s...\n", filename );
    MMMessage( msg );
    dialInputs->initialize( filename );

    //MMAddModule( dialInputs, 1, 0, 1 );
    //MMActivateModule( dialInputs );

    switchInputs = new VirtualInput( "virtualswitches" );
    MMGetStringSymbol( "virtual_switches", filename, "switches.scr" );
    sprintf( msg, "Loading switch script %s...\n", filename );
    MMMessage( msg );
    switchInputs->initialize( filename );

    //MMAddModule( switchInputs, 1, 0, 1 );
    //MMActivateModule( switchInputs );

    encoders = new VEncoder;
    analogIO = new VAnalog;
    digitalIO = new VDigital;
    timers = new VTimer;
    accels = new VAccel;
    gyros = new VGyro;
    power = new VPower;
    switches = new VSwitch;
    dials = new VDial;
    dcmotors = new VMotor;

    initStatus = true;

    MMMessage( "Hardware interface successfully initialized ( virtual ).\n" );

    MMMessage("VirtalHW::initialize() returning\n");
    return;
  } else {
    // More than one Virtual hardware object created!
    MMWarning( "VirtualHW::initialize", "Trying to initialize more than once!");
  }

}

// VirtualHW::cleanup : Removes modules etc.
void VirtualHW::cleanup( void ) {
  int count;

  if ( initStatus ) {
    // Remove the virtual motors
    for ( count = 0; count < 6; count++ ) {
      MMRemoveModule ( motors[count] );
      delete motors[count];
    }

    MMRemoveModule( dialInputs );
    delete dialInputs;
    MMRemoveModule( switchInputs );
    delete switchInputs;

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
    // This virtual hardware instance is not the first one created!
  }
}

// VirtualHW::~VirtualHW : Class destructor
VirtualHW::~VirtualHW( ) {

  if ( initStatus ) {
    // Somebody exited without proper cleanup!
    cleanup();
  }

}

CLOCK VirtualHW::readClock( void ) {
  // Currently, we use the Module Manager time steps

  return MMGetStepPeriod() * ( CLOCK( MMGetStepCount() ) + CLOCK( 1 ) );

};

void VEncoder::enable( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "VEncoder::read", "Encoder index out of bounds!" );
    return;
  }

  motors[index]->enableEncoder( true );
}

void VEncoder::disable( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "VEncoder::read", "Encoder index out of bounds!" );
    return;
  }

  motors[index]->enableEncoder( false );
}


// Reads the encoder value from the corresponding virtual motor
uint16 VEncoder::read( uint index ) {

  if ( index >= 6 ) {
    MMWarning ( "VEncoder::read", "Encoder index out of bounds!" );
    return 0;
  }

  return motors[index]->readEncoder();

}

// Resets the encoder value for the corresponding virtual motor
void VEncoder::reset( uint index ) {
  if ( index >= 6 ) {
    MMWarning ( "Vencoder::reset", "Encoder index out of bounds!" );
    return;
  }
  motors[index]->resetEncoder();
}

// Sets the analog command output value (V) for the corresponding
// virtual motor.
void VAnalog::write( uint index, float value) {
  float command;

  if ( index >= 6 ) {
    MMWarning ( "VAnalog::write", "Analog output index out of bounds!" );
    return;
  }

  // Convert the analog command to motor terminal voltage.
  command = VM_DRIVE_SUPPLY * VM_DRIVE_SCALE * ( value - VM_DRIVE_OFFSET );

  // Inform the virtual motor about the voltage
  motors[index]->setCommand( command );

}

// Reads in an analog input channel (V)
// For the time being, all analog inputs return 0.0
float VAnalog::read( uint index ) {

  if ( index >= 16 )
    MMWarning ( "VAnalog::read", "Analog input index out of bounds!" );

  return 0;
}

bool VSwitch::read( uint index ) {

  // Read the virtual inputs for the switches and decide on the
  // switch configuration with a threshold on the value
  if ( switchInputs->getChannelValue( index ) > 0.5 )
    return true;
  else
    return false;

}

float VDial::read( uint index ) {

  float value = dialInputs->getChannelValue( index );

  // Clip the value to dial range

  if ( value > 1.0 ) value = 1.0;
  if ( value < -1.0 ) value = -1.0;

  return value;
}

//
// VMotor::VMotor : Class constructor
//
VMotor::VMotor( void ) {

  int count;

  MMMessage( "Initializing DC motor interfaces..." );

  // Initialize internal fields
  for ( count = 0; count < 6; count++ ) {

    // Choose the mode of operation
    controlMode[count] = VOLTAGE_MODE;
  }

  MMMessage( "done.\n" );

}

// VMotor::getParams : Returns hardware specific constant parameters
void VMotor::getParams( uint index, MotorParam_t *params ) {

  params->torqueConstant = VM_TORQUE_CONST;
  params->speedConstant = VM_SPEED_CONST;
  params->terminalResistance = VM_RA;

  params->encRatio = VM_ENC_RATIO;
  params->gearRatio = VM_GEAR_RATIO;

}

float VMotor::getVoltage( uint index ) { 

  // Read the motor terminal voltage from the VirtualMotor class
  return motors[index]->getVoltage( ); 

};

float VMotor::getCurrent( uint index ) {

  // Read the motor armature current from the VirtualMotor class
  return motors[index]->getCurrent( ); 

};

float VMotor::getBackEMF( uint index ) {

  // Read the motor back EMF from the VirtualMotor class
  return motors[index]->getBackEMF( ); 

};

//
// Sets the command to the motor based on the control mode and the
// calibration parameters.
//
void VMotor::setCommand( uint index, float cmd ) {

  // Set the command through the VirtualMotor interface. Note that the
  // VirtualMotor does proper clipping of the command voltage.

  motors[index]->setCommand( cmd );

};

float VMotor::getCommand( uint index ) {

  // Get the latest command from the corresponding VirtualMotor module.
  return motors[index]->getCommand();

}
