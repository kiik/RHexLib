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
 * $Id: SpeedFilter.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * The Speed Filter Module. Periodically reads absolute encoder positions
 * and computes a filtered speed value.
 *
 * This file implements the member functions defined in SpeedFilter.hh
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include "StdModules.hh"
#include "SpeedFilter.hh"
#include "EncoderReader.hh"

// SpeedFilter::SpeedFilter : Class constructor
SpeedFilter::SpeedFilter( int index )
  : Module ( SPEEDFILTER_NAME, index, false, false ) {

  int   count;
  double fir[SS_FILTER_WIDTH] = {
    2.817781652883022e-02,
    1.424732848255746e-01,
    3.293488986455952e-01,
    3.293488986455952e-01,
    1.424732848255746e-01,
    2.817781652883022e-02};

  speed = 0.0;

  for ( count = 0; count < SS_FILTER_WIDTH; count++ )
    spdbuf[count] = 0;

  bptr = 0;

  enc = NULL;

  for ( count = 0; count < SS_FILTER_WIDTH; count++ )
    coeffs[ count ] = fir[ count ];
}

// SpeedFilter::init
void SpeedFilter::init( void ) {

  // Find the corresponding motor module to read the encoder count from
  if ( ( enc = (EncoderReader *) 
         MMFindModule( ENCODERREADER_NAME, getIndex() )) == NULL)
    MMFatalError ( "SpeedSensor::init", "Cannot find encoder reader module!" );

}

// SpeedFilter::activate
void SpeedFilter::activate( void ) {

  int   count;

  // Fill the buffer in with the current position values to avoid
  // spikes in position data
  speed = 0.0;

  for ( count = 0; count < SS_FILTER_WIDTH; count++ )
    spdbuf[count] = enc->getSpeed();

  bptr = 0;

  // Grab the encoder reader module
  MMGrabModule ( enc, this );
}

// SpeedFilter::deactivate
void SpeedFilter::deactivate( void ) {

  // Release the encoder reader module
  MMReleaseModule ( enc, this );
}

// SpeedFilter::update
void SpeedFilter::update( void ) {

  int   count, curptr;
  float sum = 0;

  // read the current encoder value in the appropriate buffer location
  spdbuf[bptr] = enc->getSpeed();

  // Compute the filtered speed
  curptr = bptr;
  for ( count = 0; count < SS_FILTER_WIDTH; count++ ) {
    sum += coeffs[count] * spdbuf[curptr];
    if ( (curptr = curptr - 1) < 0)
      curptr = SS_FILTER_WIDTH-1;
  }
 
  // Update the buffer pointer
  if ( (bptr = bptr + 1) >= SS_FILTER_WIDTH)
    bptr = 0;

  speed = sum;

  //printf ( "SpeedFilter[%i] value: %e\n", getIndex(), speed );

}


