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
 * $Id: EncoderReader.hh,v 1.4 2001/07/24 02:06:29 ulucs Exp $
 *
 * The Encoder Reader Module
 *
 * This file contains the definition of the EncoderReader class 
 * and all the associated definitions. The member function definitions
 * are in EncoderReader.cc
 *
 *  The Encoder Reader module periodically reads the encoder values and 
 * keeps track of the absolute output shaft position, taking the
 * gearhead into account.
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _ENCODERREADER_HH
#define _ENCODERREADER_HH

// Local includes
#include "ModuleManager.hh"
#include "Hardware.hh"
#include "GenericSensor.hh"

// Module specific constants ---------------------------------------

// The EncoderReader class -----------------------------------------
class EncoderReader : public Module {

public:
  EncoderReader ( int index );

  // Module base class interface
  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { encoders->enable( getIndex() ); };
  void  deactivate ( void ) { encoders->disable( getIndex() ); };
  void  update ( void );

  // EncoderReader interface
  void      reset ( double pos );
  void      resetRelPosition ( void ) { relpos = 0.0; };

  float     getPosition ( void ) { return curpos; }; // Range: [-PI, PI]
  float     getRelPosition ( void ) { return relpos; }; // cumulative

  float     getSpeed ( void ) { return curspeed; };
  double    getTimestamp ( void ) { return timestamp; };

private:
  // Current absolute motor output shaft position (rad) and speed (rad/s)
  double   curpos;  // range [-pi, pi]
  double   relpos;  // cumulative since last resetAbsPos().
  double   curspeed;

  // Timestamp for the latest reading (ns)
  double  timestamp;

  // Data from the last update
  uint16  lastenc;
  double  lastpos;
  double  lasttime;

  // General Access to the current hardware
  Hardware   *hardware;
  // Quick Access to hardware components
  EncoderHW  *encoders;
  // Constant motor parameters. Needed for speed and absolute position
  // computations.
  DCMotorHW::MotorParam_t  motorparams;

};

#endif
