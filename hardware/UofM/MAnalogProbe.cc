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
 * $Id: MAnalogProbe.cc,v 1.3 2001/08/05 18:13:24 ulucs Exp $
 *
 * The MAnalogProbe module. Handles the analog inputs in the Michigan Hardware.
 *
 * Created       : Uluc Saranli, 01/19/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "io/mpc550.hh"
#include "MichiganHW.hh"
#include "MComponents.hh"
#include "MAnalogProbe.hh"

// MAnalogProbe::MAnalogProbe : Class constructor
MAnalogProbe::MAnalogProbe( uint numchan, bool polling ) 
  : Module( "manalogprobe", 0, false, polling ) {

  channels = numchan;

  controlWord = 0x40 | ADIO_RANGE5V | ADIO_UNIPOLAR;

  curChannel = lastChannel = 0;
  stepMark = 0;
  convCount = 0;

  conversion = false;

  memset( value, 0, 16 * sizeof( float ) );
  memset( timestamp, 0, 16 * sizeof( CLOCK ) );

}

void MAnalogProbe::update( void ) {

  if ( curChannel >= channels ) {

    if ( MMGetStepCount() == stepMark ) {

      // Done with all the channels and waiting for a new module manager step.
      // meaningful only when the module is polling
      return;

    } else {
      // New module manager step. We can restart acquisition of the channels
      curChannel = 0;
      stepMark = MMGetStepCount();
    }
  } 

  // Still not done with all the channels for the current run. Check pending 
  // conversions and issue a new one if necessary

  if ( conversion ) {
    // There is a pending conversion, so try to read the data.

    if ( MPC550Card->checkADC( 1 ) ) {
      // The conversion was from AD1

      value[ lastChannel ] = 5.0 * MPC550Card->readADC( 1 ) / 0xfff;
      conversion =  false;

    } else if ( MPC550Card->checkADC( 2 ) ) {
      // The conversion was from AD2

      value[ lastChannel ] = 5.0 * MPC550Card->readADC( 2 ) / 0xfff;
      conversion = false;

    }

    if ( !conversion ) {
      timestamp[ lastChannel ] = MMReadClock();
      curChannel++;
    }
  }

  if ( !conversion && ( curChannel < channels ) ) {
    // If the previous conversion is done and there are still more channels
    // to go, start another conversion

    if ( curChannel < 8 ) {
      MPC550Card->acquireADC( 1, controlWord, curChannel );
    } else {
      MPC550Card->acquireADC( 2, controlWord, curChannel - 8 );
    }
    //  printf( "Starting conversion of channel %i\n", curChannel );
    conversion = true;
    lastChannel = curChannel;
  }
}


