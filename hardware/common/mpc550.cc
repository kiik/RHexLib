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
 * $Id: mpc550.cc,v 1.3 2001/08/05 18:13:24 ulucs Exp $
 *
 * This file implements the functions defined in mpc550.hh
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include "rhexio.hh"
#include "types.hh"
#include "io/mpc550.hh"

// mpc550::mpc550 : Class constructor
mpc550::mpc550( uint16 io, uint timer_i, uint adc_i ) {

  iobase = io;
  timerirq = timer_i;
  adcirq = adc_i;

  timerInitial[0] = timerInitial[1] = timerInitial[2] = 0;

  // Reset the ADC by reading the ports
  readADC( 1 );
  readADC( 2 );

}

void mpc550::setDIODir( bool PortA, bool PortB, bool PortCH, bool PortCL ) {
  uint8 control = 0x80;

  if ( !PortA )
    control |= 0x90;

  if ( !PortB )
    control |= 0x02;

  if ( !PortCH )
    control |= 0x08;

  if ( !PortCL )
    control |= 0x01;
    
  routp( control, ADIO_DIOCNTRL );
}

void mpc550::acquireADC( uint ADCnum, uint16 control, uint channel ) {

  int   ADCaddr;
  
  if ( ADCnum == 1 ) {
    ADCaddr = ADIO_ADC1;
  } else if ( ADCnum == 2 ) {
    ADCaddr = ADIO_ADC2;
  } else    
    return;

  if ( channel > 7 )
    return;

  routp( ( control & 0xfff8 ) | ( channel & 0x7 ), ADCaddr );
}

bool mpc550::checkADC( uint ADCnum ) {
  
  if ( ADCnum == 1 ) {
    return bool( rinp( ADIO_ADPOLL ) & ADIO_ADC1EOC );
  } else if ( ADCnum == 2 ) {
    return bool( rinp( ADIO_ADPOLL ) & ADIO_ADC2EOC );
  } else    
    return false;

}

uint16 mpc550::readADC( uint ADCnum ) {

  if ( ADCnum == 1 ) {
    return rinpw( ADIO_ADC1 );
  } else if ( ADCnum == 2 ) {
    return rinpw( ADIO_ADC2 );
  } else    
    return false;

  
}

void mpc550::clearTimerInt( uint timer ) {
  if ( timer > 1 ) return;
  
  routp( 0x00, ADIO_CLRINTR1 + timer );
  
  return;
}

// mpc550::setupTimer : Configures a timer for a particular frequency.
// Note that hertz=0 sets up the slowest possible count
void mpc550::setupTimer( uint timer, uint mode, uint32 hertz ) {

  uint8 control = 0x30;
  uint8 lsb = 0;
  uint8 msb = 0;
  
  if ( timer > 2 ) return;
  if ( mode > 5 ) return;
  // If hertz == 0, Choose count = 0xffff for slowest counter operation
  if ( hertz == 0 )
    timerInitial[ timer ] = 0xffff;
  else if ( ( hertz < 110 ) || ( hertz > MPC550_FREQ ) ) return;
  else 
    timerInitial[ timer ] = MPC550_FREQ / hertz;

  lsb = ( timerInitial[ timer ] ) & 0x00FF;
  msb = ( timerInitial[ timer ] & 0xFF00) >> 8 ;

  control |= ( mode << 1 );
  control |= ( timer << 6 );
  
  //    disable();
  
  routp( control, ADIO_TMRCNTRL );
  routp( lsb, ADIO_TMRCNTR0 + timer );
  routp( msb, ADIO_TMRCNTR0 + timer );
  
  //    enable();
  
  return;
}

void mpc550::timerInfo( uint timer, uint *initial, uint *clockFreq ) {

  *initial = timerInitial[ timer ];

  *clockFreq = MPC550_FREQ;

}
