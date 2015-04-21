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
 * $Id: mpc550.hh,v 1.1 2001/08/05 18:13:24 ulucs Exp $
 *
 * All data type definitions and function prototypes for the mpc550
 * Analog/Digital IO card low level interface.
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _MPC550_HH
#define _MPC550_HH


// Local includes
#include "types.hh"
#include "rhexio.hh"

// Constants ------------------------------------------------------

#define ADIO_DIOPORTA (iobase)
#define ADIO_DIOPORTB (iobase + 0x01)
#define ADIO_DIOPORTC (iobase + 0x02)
#define ADIO_DIOCNTRL (iobase + 0x03)

#define ADIO_TMRCNTR0 (iobase + 0x04)
#define ADIO_TMRCNTR1 (iobase + 0x05)
#define ADIO_TMRCNTR2 (iobase + 0x06)
#define ADIO_TMRCNTRL (iobase + 0x07)

#define ADIO_ADC1     (iobase + 0x08)
#define ADIO_ADC1HIGH (iobase + 0x09)
#define ADIO_ADC2     (iobase + 0x0A)
#define ADIO_ADC2HIGH (iobase + 0x0B)
#define ADIO_ADPOLL   (iobase + 0x0C)
#define ADIO_ADC1EOC  (0x80)
#define ADIO_ADC2EOC  (0x40)

#define ADIO_CLRINTR1 (iobase + 0x0C)
#define ADIO_CLRINTR2 (iobase + 0x0D)

// DAC commands and registers
#define ADIO_LOADDAC  (iobase + 0x0E)
#define ADIO_DAC1     (iobase + 0x10)
#define ADIO_DAC2     (iobase + 0x18)

// Analog input range flags
#define ADIO_RANGE5V  0x00
#define ADIO_RANGE10V 0x10
#define ADIO_UNIPOLAR 0x00
#define ADIO_BIPOLAR  0x20

#define ADIO_LSB 0
#define ADIO_MSB 1

// MPC550 Timer freq. in Hz (format should be long int!)
#define MPC550_FREQ     (7159000L)

// The mpc550 class ----------------------------------------------

class mpc550 {
 private:
  uint16  iobase;
  uint    timerirq;
  uint    adcirq;

  uint16  timerInitial[3];

public:
  mpc550( uint16 io, uint timer_im, uint adc_i );

  // Digital IO facilities
  void   writePortA( uint8 val ) { routp( val, ADIO_DIOPORTA ); };
  uint   readPortA(void) { return rinp( ADIO_DIOPORTA ); };
  void   writePortB( uint8 val ) { routp( val, ADIO_DIOPORTB ); };
  uint   readPortB(void) { return rinp( ADIO_DIOPORTB ); };
  void   writePortC( uint8 val ) { routp( val, ADIO_DIOPORTC ); };
  uint   readPortC( void ) { return rinp( ADIO_DIOPORTC ); };
  void   setDIODir( bool PortA, bool PortB, bool PortCH, bool PortCL );

  // Analog output facilities
  void   analogOut( uint chan, uint16 value );

  // Analog input facilities
  void   acquireADC( uint ADCnum, uint16 control, uint channel );
  bool   checkADC( uint ADCnum );
  uint16 readADC( uint ADCnum );

  // Timer facilities
  void   clearTimerInt( uint timer );
  void   setupTimer( uint timer, uint mode, uint32 hertz );
  uint16 readTimer( uint timer );
  void   timerInfo( uint timer, uint *initial, uint *clockFreq );

};

// mpc550::analogOut : Output a DAC value
inline void mpc550::analogOut( uint chan, uint16 value ) {
  int IObase;

  // Clip the values into appropriate ranges
  chan &= 0x07;
  value &= 0x0fff;
  
  if ( chan < 4 )
    IObase = ( ADIO_DAC1 | ( chan << 1 ) );
  else
    IObase = ( ADIO_DAC2 | ( chan << 1 ) );
  
  // write lower 8 bits
  routp( value & 0x00FF, IObase | ADIO_LSB );
  // write upper 4 bits
  routp( (value & 0x0F00) >> 8, IObase | ADIO_MSB );
  
  // send the data
  routp( 0x0, ADIO_LOADDAC );

}

inline uint16 mpc550::readTimer( uint timer ) {
  int lsb, msb;
  
  routp( ( timer << 6 ), ADIO_TMRCNTRL );
  
  lsb = rinp( ADIO_TMRCNTR0 + ( timer ) );
  msb = rinp( ADIO_TMRCNTR0 + ( timer ) );
  
  return uint16( ( msb << 8 ) + lsb );
}


#endif
