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
 * $Id: SpeedFilter.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * The Motor Speed Filter Module
 *
 * This file contains the definition of the SpeedFilter module class 
 * and all the associated definitions. The member function definitions
 * are in SpeedFilter.cc
 *
 *  The Speed Filter module periodically reads absolute encoder positions and 
 * provides a filtered speed measurement of the motor output shaft
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _SPEEDFILTER_HH
#define _SPEEDFILTER_HH

#include "ModuleManager.hh"

class EncoderReader;

// Module specific constants ---------------------------------------

// Number of samples to use in the filtering 
#define SS_FILTER_WIDTH     6

// The SpeedFilter class -----------------------------------------
class SpeedFilter : public Module {

public:
  SpeedFilter ( int index );

  // Module base claass interface
  void  init ( void );
  void  uninit ( void ) { };
  void  activate ( void );
  void  deactivate ( void );
  void  update ( void );

  // Interface local to the class
  float getSpeed ( void ) { return speed; };

private:
  // Coefficients for the filter design
  float coeffs[SS_FILTER_WIDTH];
  
  // Latest computation of speed
  float speed;

  // Buffers for encoder position and timestamps
  float spdbuf[SS_FILTER_WIDTH];

  // index of the current sample in the buffers above
  int   bptr;

  EncoderReader *enc;

};

#endif
