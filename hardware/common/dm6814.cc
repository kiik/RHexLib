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
 * $Id: dm6814.cc,v 1.3 2001/08/05 18:13:24 ulucs Exp $
 *
 * This file implements the functions defined in dm6814.hh
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include "io/dm6814.hh"

dm6814::dm6814( uint16 io, uint i ) {
  iobase = io;
  irq = i;

  mode[0] = mode[1] = mode[2] = 0;

  IRQreg = 0;
}

// dm6814::setupTimer : Configures a timer for a particular frequency.
// Note that hertz=0 sets up the slowest possible count
void dm6814::setupTimer( uint timer, uint mode, uint32 hertz) {

  uint8  control = 0x30;
  uint8  lsb = 0;
  uint8  msb = 0;

  if ( timer > 2 ) return;
  if ( mode > 5 ) return;
  // If hertz == 0, Choose count = 0xffff for slowest counter operation
  if ( hertz == 0 )
    timerInitial[ timer ] = 0xffff;
  else if ( ( hertz < 110 ) || ( hertz > DM6814_FREQ ) ) return;
  else 
    timerInitial[ timer ] = DM6814_FREQ / hertz;

  lsb = ( timerInitial[ timer ] ) & 0x00FF;
  msb = ( timerInitial[ timer ] & 0xFF00 ) >> 8 ;

  control |= (mode << 1);
  control |= (timer << 6);
  routp( control, iobase + ENC_TIMER_CTRL );
  routp( lsb, iobase + ENC_TIMER_CLCK + timer );
  routp( msb, iobase + ENC_TIMER_CLCK + timer );
  
  return;
}

void dm6814::timerInfo( uint timer, uint *initial, uint *clockFreq ) {

  *initial = timerInitial[ timer ];
  *clockFreq = DM6814_FREQ;

}

