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
 * $Id: StallSensor.cc,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The StallSensor Module.
 *
 * This file implements the member functions defined in StallSensor.hh
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include <math.h>
#include "StdModules.hh"
#include "StallSensor.hh"
#include "EncoderReader.hh"

StallSensor::StallSensor( int index )
  : Module ( STALLSENSOR_NAME, index, false, false ) {

  enc = NULL;
  mark = 0;

  stallFlag = false;

  tolerance = DEF_STALL_TOLERANCE;
  timeout = DEF_STALL_TIMEOUT;
}

// StallSensor::init
void StallSensor::init( void ) {

  // Find the corresponding encoder sensor module
  if ( ( enc = (EncoderReader *) 
         MMFindModule( ENCODERREADER_NAME, getIndex() )) == NULL)
    MMFatalError ( "StallSensor::init", "Cannot find encoder reader module!" );

}


// StallSensor::activate
void StallSensor::activate( void ) {

  MMGrabModule( enc, this );

  mark = MMReadTime();
}

// StallSensor::deactivate
void StallSensor::deactivate( void ) {

  MMReleaseModule( enc, this );
}

// StallSensor::update : Checks for motor stall
void StallSensor::update( void ) {
  float speed = enc->getSpeed();
  double now = MMReadTime();

  if ( ( fabs( speed ) <= tolerance ) && ( now - mark > timeout) )
    stallFlag = true;
  else if ( fabs( speed ) > tolerance ) {
    mark = now;
    stallFlag = false;
  }
}
