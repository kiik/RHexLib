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
 * $Id: StallSensor.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The Stall Sensor Module
 *
 * This file contains the definition of the StallSensor module class 
 * and all the associated definitions. The member function definitions
 * are in StallSensor.cc
 *
 *  The StallSensor module detects whether the associated axis has 
 * stalled or not. Stalling is defined as the position of the motor remaining
 * constant up to a certain tolerance for a certain period of time, both of
 * which are configurable.
 *
 * Created       : Uluc Saranli, 12/28/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _STALLSENSOR_HH
#define _STALLSENSOR_HH

#include "ModuleManager.hh"

class EncoderReader;

// Module specific constants ---------------------------------------

// Speed tolerance to indicate stall
#define DEF_STALL_TOLERANCE 5e-4;
// Required stall duration for positive answer ( in seconds )
#define DEF_STALL_TIMEOUT 0.5

// The StallSensor class -------------------------------------------

class StallSensor : public Module {
private:
  EncoderReader *enc;
  double mark;

  bool stallFlag;

  double tolerance;
  double timeout;

public:
  StallSensor ( int index );

  void init ( void );
  void uninit ( void ) { };
  void activate ( void );
  void deactivate ( void );
  void update ( void );

  void reset( void ) { mark = MMReadTime(); stallFlag = false; };
  bool read( void ) { return stallFlag; };

  void setup( double tol, double to ) { tolerance = tol; timeout = to; };
};

#endif
