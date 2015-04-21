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

/** @file UTAnalogProbe.h
 *  @brief The UTAnalogProbe module. Handles the analog inputs in the University of Tartu Hardware.
 *
 *  @author Meelik Kiik
 *  @version 0.1
 *  @date 21/04/2015 16:00
 *
 *  Created at 21/04/2015 16:00
 */

#ifndef _UTANALOGPROBE_H
#define _UTANALOGPROBE_H

#include "ModuleManager.hh"


// Module specific constants --------------------------------------

// The UTAnalogProbe class -----------------------------------------

class UTAnalogProbe : public Module {
private:

  // Config data
  uint channels;
  uint16 controlWord;

  // Dynamic data
  uint  curChannel;
  uint  lastChannel;
  bool conversion;
  MM_STEP stepMark;

  uint32 convCount;

  float value[16];
  CLOCK timestamp[16];

public:
  UTAnalogProbe( uint numchan, bool polling );

  void  init ( void ) { };
  void  uninit ( void ) { };
  void  activate ( void ) { };
  void  deactivate ( void ) { };
  void  update ( void );

  void setup( uint numchan );
  float getValue ( uint chan ) { 
    return ( chan < channels) ? value[chan] : 0.0; 
  };
  CLOCK getTimestamp ( uint chan ) { return timestamp[chan]; };
  uint32 getCount( void ) { return convCount; }
};

#endif
