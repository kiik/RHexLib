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
 * $Id: McDigital.cc,v 1.3 2001/08/05 18:13:24 ulucs Exp $
 *
 * Low level hardware interface to the Digital IO facilities of the McGill
 * version of RHex
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <math.h>
#include "ModuleManager.hh"
#include "McGillHW.hh"
#include "McComponents.hh"
#include "io/dm6814.hh"
#include "io/mpc550.hh"

McDigital::McDigital( void ) {
  int byte;

  MMMessage( "Initializing Digital IO components..." );

  // Set the MPC550 Digital IO directions 
  // Port A - output,  Port B - input
  // Port C  bits 0-3 output
  // Port C  bits 4-7 input
  MPC550Card->setDIODir( true, false, false, true);
  // Set all the DM6814 digital channels to input
  DM6814Card[0]->setDIODir( 0, false, false );
  DM6814Card[0]->setDIODir( 1, false, false );
  DM6814Card[0]->setDIODir( 2, false, false );
  DM6814Card[1]->setDIODir( 0, false, false );
  DM6814Card[1]->setDIODir( 1, false, false );
  DM6814Card[1]->setDIODir( 2, false, false );

  // Reset all the digital outputs
  for ( byte = 0; byte < 9; byte++ ) {
    setByte( byte, 0x00 );
  }

  MMMessage( "done.\n" );
}

McDigital::~McDigital( void ) {
  int byte;

  // Set the MPC550 Digital IO directions to input
  MPC550Card->setDIODir( false, false, false, false);
  // Set all the DM6814 digital channels to input
  DM6814Card[0]->setDIODir( 0, false, false );
  DM6814Card[0]->setDIODir( 1, false, false );
  DM6814Card[0]->setDIODir( 2, false, false );
  DM6814Card[1]->setDIODir( 0, false, false );
  DM6814Card[1]->setDIODir( 1, false, false );
  DM6814Card[1]->setDIODir( 2, false, false );

  // Reset all the digital outputs
  for ( byte = 0; byte < 9; byte++ ) {
    setByte( byte, 0x00 );
  }
}

// McDigital::getByte : Reads in a byte of digital inputs.
uint McDigital::getByte( uint byte ) {

  switch ( byte ) {
  case 0:
    // MPC550 Port A
    return MPC550Card->readPortA();

  case 1:
    // MPC550 Port B
    return MPC550Card->readPortB();

  case 2:
    // MPC550 Port C
    return MPC550Card->readPortC();

  case 3:
    // DM6814 #1 Chip #1
    return DM6814Card[0]->readDIO( 0 );

  case 4:
    // DM6814 #1 Chip #2
    return DM6814Card[0]->readDIO( 1 );

  case 5:
    // DM6814 #1 Chip #3
    return DM6814Card[0]->readDIO( 2 );

  case 6:
    // DM6814 #2 Chip #1
    return DM6814Card[1]->readDIO( 0 );

  case 7:
    // DM6814 #2 Chip #2
    return DM6814Card[1]->readDIO( 1 );

  case 8:
    // DM6814 #2 Chip #3
    return DM6814Card[1]->readDIO( 2 );

  default:
    // Warning!: Invalid byte index. This should not happen!
    MMWarning( "McDigital::getByte", "Byte index out of bounds!" );
    return 0;
  }
  
}

// McDigital::setByte : Writes out a byte of digital outputs.
void McDigital::setByte( uint byte, uint8 value ) {
  switch ( byte ) {
  case 0:
    // MPC550 Port A
    MPC550Card->writePortA( value );
    break;

  case 1:
    // MPC550 Port B
    MPC550Card->writePortB( value );
    break;

  case 2:
    // MPC550 Port C
    MPC550Card->writePortC( value );
    break;

  case 3:
    // DM6814 #1 Chip #1
    DM6814Card[0]->writeDIO( 0, value );
    break;

  case 4:
    // DM6814 #1 Chip #2
    DM6814Card[0]->writeDIO( 1, value );
    break;

  case 5:
    // DM6814 #1 Chip #3
    DM6814Card[0]->writeDIO( 2, value );
    break;

  case 6:
    // DM6814 #2 Chip #1
    DM6814Card[1]->writeDIO( 0, value );
    break;

  case 7:
    // DM6814 #2 Chip #2
    DM6814Card[1]->writeDIO( 1, value );
    break;

  case 8:
    // DM6814 #2 Chip #3
    DM6814Card[1]->writeDIO( 2, value );
    break;

  default:
    // Warning!: Invalid byte index. This should not happen!
    MMWarning( "McDigital::setByte", "Byte index out of bounds!" );
    break;
  }

}

// McDigital::getBit : Reads in a digital channel.
bool McDigital::getBit( uint byte, uint bit ) {

  return bool( ( getByte( byte ) & ( 1 << bit ) ) >> bit );

}

// McDigital::setBit : Writes out a digital channel.
void McDigital::setBit( uint byte, uint bit, bool value ) {

  setByte( byte, ( (getByte( byte ) & ~(1 << bit)) | (value << bit) ) );

}


