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
 * $Id: MichiganHW.cc,v 1.5 2001/08/05 18:13:24 ulucs Exp $
 *
 * Low level hardware interface to the virtual hardware components
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// System includes
#include <math.h>

// Local includes
#include "config.hh"
#include "ModuleManager.hh"
#include "MichiganHW.hh"
#include "MComponents.hh"
#include "io/dm6814.hh"
#include "io/mpc550.hh"

// Start in the uninitialized state
bool MichiganHW::initStatus = false;
bool MichiganHW::clockStatus = false;

// Global variables for general access
dm6814 *DM6814Card[2];
mpc550 *MPC550Card;

// MichiganHW::MichiganHW : Class constructor Creates and initializes
// the virtual motors and creates the virtual hardware component
// interfaces
MichiganHW::MichiganHW( void ) {

  if ( !clockStatus ) {

    // Initialize the system clock
    initClock();

    clockStatus = true;
  } else {
    // Warning!: More than one Michigan Hardware object created!
    MMWarning( "MichiganHW::MichiganHW", 
               "More than one MichiganHW object created!" );
  }
}

void MichiganHW::initialize( void ) {

  if ( !initStatus ) {
    // This is the first time a Michigan Hardware object is initialized

    MMMessage( "Initializing low level hardware interface ( Michigan )...\n" );

    // Allocate instances of the low level PC104 card interfaces
    DM6814Card[0] = new dm6814( ENCI_IOBASE, ENCI_IRQ );
    DM6814Card[1] = new dm6814( ENCII_IOBASE, ENCII_IRQ );
    MPC550Card = new mpc550( ADIO_IOBASE, ADIO_TIMER_IRQ, ADIO_ADC_IRQ);

    // Allocate the components of the hardware interface
    encoders = new MEncoder;         // Encoder interfaces
    analogIO = new MAnalog;          // Analog IO
    digitalIO = new MDigital;        // Digital IO
    timers = new MTimer;             // Timer operations
    accels = new MAccel( this );     // Accelerometers
    power = new MPower( this );      // Power measurement
    switches = new MSwitch( this );  // DIP switches
    dials = new MDial( this );       // RC inputs
    dcmotors = new MMotor( this );   // DC motors

    initStatus = true;
    driveStatus = 0x00;

    MMMessage( "Hardware interface successfully initialized ( Michigan ).\n" );

  } else {
    // Warning!: More than one Michigan Hardware object created!
    MMWarning( "MichiganHW::initialize", 
               "Trying to initialize more than once!" );
  }
}

void MichiganHW::cleanup( void ) {

  if ( initStatus ) {

    MMMessage( "Cleaning up low level hardware interface ( Michigan )...\n" );

    if ( encoders != NULL ) delete encoders;
    if ( analogIO != NULL ) delete analogIO;
    if ( digitalIO != NULL ) delete digitalIO;
    if ( timers != NULL ) delete timers;
    if ( accels != NULL ) delete accels;
    if ( power != NULL ) delete power;
    if ( switches != NULL ) delete switches;
    if ( dials != NULL ) delete dials;
    if ( dcmotors != NULL ) delete dcmotors;

    // Free instances of the low level PC104 card interfaces
    if ( DM6814Card[0] != NULL) delete DM6814Card[0];
    if ( DM6814Card[1] != NULL) delete DM6814Card[1];
    if ( MPC550Card != NULL) delete MPC550Card;

    initStatus = false;
    clockStatus = false;

  } else {
    // Warning!: This Michigan Hardware instance is not the first one
    // created! */
  }
}

// MichiganHW::~MichiganHW : Class deconstructor
MichiganHW::~MichiganHW( void ) {

  if ( initStatus ) {
    // Somebody deleted the object without calling cleanup()!
    cleanup();
  }

}

// MichiganHW::driveEnable : Enable / Disable the motor drives
void MichiganHW::driveEnable( bool enable ) {

  if ( enable ) {

    // Bring the drive enable line high
    digitalIO->setBit( DRIVE_ENABLE >> 3, DRIVE_ENABLE & 0x07, true );

    // TODO: This needs to change somehow!! The delay() calls are
    // unacceptable!

    rdelay( 10 ); // Wait for a little while

    // Send the drive start pulse
    digitalIO->setBit( DRIVE_START >> 3, DRIVE_START & 0x07, true );
    rdelay( 10 ); // Wait for a little while
    digitalIO->setBit( DRIVE_START >> 3, DRIVE_START & 0x07, false );
    rdelay( 10 ); // Wait for a little while.

    //    driveStatus = true;

  } else {
    // Bring the drive enable line low 
    digitalIO->setBit( DRIVE_ENABLE >> 3, DRIVE_ENABLE & 0x07, false );

    //    driveStatus = false;
  }
}

// MichiganHW::driveEnable : Enable / Disable the motor drives
void MichiganHW::driveEnable( uint index, bool enable ) {

  if ( index > 5) return;

  if ( enable ) {

    // Enable the drive circuitry is this is the first time
    if ( !driveStatus )
      driveEnable( true );

    // Update the drive enable bits
    driveStatus |= ( 1 << index );

  } else {

    // Update the drive enable bits
    driveStatus &= ~( 1 << index );

    // Disable the drive circuitry is this was the last axis
    if ( driveStatus == 0x00 ) {
      driveEnable( false );
    }
  }
}
