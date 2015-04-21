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
 * $Id: PulseWidth.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * The PulseWidth  Module. Measures PWM signals through timers.
 *
 * This file contains the Module class definition, all datatypes
 * and constants associated with the PulseWidth module.
 *
 * Created       : Uluc Saranli, 12/27/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _PULSEWIDTH_HH
#define _PULSEWIDTH_HH

#include "ModuleManager.hh"
#include "Hardware.hh"

// Module specific constants ---------------------------------------

// The PulseWidth class --------------------------------------------

class PulseWidth : public Module {
private:

  // Indices of the associated timer and digital input line
  uint timerId;
  uint byteId;
  uint bitId;

  DigitalHW *digitalIO;
  TimerHW   *timers;

  uint   timerFreq;
  uint   timerInitial;
  double timeout;

  float  pulseWidth;
  double timestamp;

  double lastGate; 

public:
  PulseWidth( uint timer, uint digital, double timeout, bool polling );
  PulseWidth( uint timer, uint digital, double timeout, bool polling, 
              Hardware *hw );

  void  init ( void );
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void );

  float getWidth ( void ) { return pulseWidth; };
  double getTimestamp ( void ) { return timestamp; };

};


#endif
