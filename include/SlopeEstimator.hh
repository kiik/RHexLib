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
 * $Id: SlopeEstimator.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Uluc Saranli, 07/05/2001
 * Last Modified : Uluc Saranli, 07/05/2001
 *
 ********************************************************************/

#ifndef _SLOPEESTIMATOR_HH
#define _SLOPEESTIMATOR_HH

// Local includes
#include "basicmath.hh"
#include "ModuleManager.hh"
#include "WalkMachine.hh"
#include "Filter.hh"
#include "Hardware.hh"
#include "GenericSensor.hh"

// Module specific constants ---------------------------------------

// ------------------------------------------------------------------
// BodyPitchSensor class. Provides the body pitch in radians ------- 

class BodyPitchSensor : public GenericSensor {

public:
  BodyPitchSensor( Hardware *hw ) { accels = hw->accels; };

  float value( void ) { 
    return acc2pitch_conversion( accels->read( AccelHW::AXIS_Y ) );
  };

private:
  AccelHW *accels;

  // Converts forward acceleration to pitch estimate, assuming stationary 
  // motion. This needs to be filtered to give accurate average pitch
  // estimation.
  double  acc2pitch_conversion( double acc ) { 
    return asin( saturate( acc / 9.80665, -1.0, 1.0 ) );
  };
};

// ------------------------------------------------------------------
// The SlopeEstimator class -----------------------------------------

class SlopeEstimator : public Module {

public:
  SlopeEstimator( void );

  // Module base class interface
  void init ( void );
  void uninit ( void );
  void activate ( void ) { MMGrabModule( pitchFilter, this ); };
  void deactivate ( void ) { MMReleaseModule( pitchFilter, this ); };
  void update ( void ) { slopeEstimate = pitchFilter->output(); };

  float readSlope( void ) { return slopeEstimate; };

private:

  float slopeEstimate;

  // Inclination estimation related stuff
  BodyPitchSensor  *rawPitchSensor;
  Filter           *pitchFilter;
  FilterParam_t     pitchFilterParams;

};

#endif
