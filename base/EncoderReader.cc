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
 * $Id: EncoderReader.cc,v 1.4 2001/07/24 02:06:29 ulucs Exp $
 *
 * The Encoder Reader Module. Periodically reads encoders and keeps
 * track of absolute position
 *
 * This file implements the member functions defined in EncoderReader.hh
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "StdModules.hh"
#include "EncoderReader.hh"
#include "sysutil.hh"

// EncoderReader::EncoderReader : Class constructor
EncoderReader::EncoderReader( int index )
  : Module ( ENCODERREADER_NAME, index, false, false ) {

  hardware = MMGetHardware();

  if ( hardware->encoders == NULL)
    MMFatalError ( "EncoderReader::EncoderReader", 
                   "Encoder hardware component not supported!" );

  if ( hardware->dcmotors == NULL)
    MMFatalError ( "EncoderReader::EncoderReader", 
                   "DC Motor hardware component not supported!" );

  curpos = relpos = curspeed = lastpos = 0.0;
  timestamp = lasttime = 0;

  lastenc = 0;

  encoders = hardware->encoders;
  hardware->dcmotors->getParams( getIndex(), &motorparams );
}

// EncoderReader::update
void EncoderReader::update( void ) {

  uint16 enc;
  int16  diff;
  double  posdiff, twoPi = 2.0 * M_PI;
   
  // Read the current encoder count
  enc = encoders->read( getIndex() );

  // Compute the change from the last reading
  diff = enc - lastenc;

  // Record stale data
  lastenc = enc;
  lastpos = curpos;
  lasttime = timestamp;

  // Update the current motor position and timestamp
  posdiff = twoPi * diff * motorparams.gearRatio / motorparams.encRatio;
  curpos += posdiff;
  relpos += posdiff;
  if ( curpos >= M_PI ) curpos -= twoPi;
  else if ( curpos < -M_PI ) curpos += twoPi;

  timestamp = MMReadUTime();

  // Compute the current speed
  if ( timestamp > lasttime )
    curspeed = ( posdiff ) / ( timestamp - lasttime );
  else
    curspeed = 0;

  //printf ( "Encoder[%i] position: %e, speed:%e, time: %li\n", 
  //         getIndex(), curpos, curspeed, timestamp );
}

// EncoderReader::reset: Resets the current position value to 0
void EncoderReader::reset( double pos ) {

  float twoPi = 2.0 * M_PI;

  // Read the current encoder count to avoid unwanted position 
  // change in the next update
  lastenc = encoders->read( getIndex() );

  pos = pos - floor( pos / twoPi ) * twoPi;
  if ( pos >= M_PI ) curpos -= twoPi;

  // Reset the current motor position and speed
  lastpos = curpos = pos;
  curspeed = 0;

  // Set the timestamp
  timestamp = MMReadUTime();

}
