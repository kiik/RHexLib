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
 * $Id: SlopeEstimator.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Uluc Saranli, 07/05/2001
 * Last Modified : Uluc Saranli, 07/05/2001
 *
 ********************************************************************/

#include "SlopeEstimator.hh"
#include "StdModules.hh"

SlopeEstimator::SlopeEstimator( void ) 
  : Module( SLOPEESTIMATOR_NAME, 0, false, false ) {

  rawPitchSensor = NULL;

  pitchFilter = NULL;
}

void SlopeEstimator::init( void ) {

  Hardware *hw = MMGetHardware();

  // Create a raw pitch sensor
  rawPitchSensor = new BodyPitchSensor( hw );

  // Create a filter with appropriate parameters for the online pitch
  // estimation
  pitchFilterParams.Ftype  = BUTTERWORTH;
  pitchFilterParams.Fc     = 4;
  pitchFilterParams.Fs     = 1000;
  pitchFilterParams.Gp     = 1;
  pitchFilterParams.parameter[0] = 4;

  pitchFilter = new Filter( "walkpitchfilter", 0 );

  if ( pitchFilter->Design( &pitchFilterParams ) != 0 )
    MMFatalError( "SlopeEstimator::init", "Could not design pitch filter..." );

  // Assign the body pitch sensor as a source to the filter.
  pitchFilter->AssignSource( rawPitchSensor );

  // Add and activate the filter module with periodicity 10.
  MMAddModule( pitchFilter, 10, 0, USER_DEFINED_SENSORS );
}

void SlopeEstimator::uninit( void ) {

  // Remove the pitch filter module
  MMRemoveModule( pitchFilter );

  // Delete the raw pitch sensor
  if ( rawPitchSensor ) delete rawPitchSensor;
  rawPitchSensor = NULL;

  // Delete the pitch filter
  if ( pitchFilter ) delete pitchFilter;
  pitchFilter = NULL;

}

