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
 * $Id: dm6814.hh,v 1.1 2001/08/05 18:13:24 ulucs Exp $
 *
 * All data type definitions and function prototypes for the dm6814
 * encoder card low level interface.
 *
 * The dm6814 class captures all the register level functionality of
 * the RTDUSA DM6814 encoder interface card. Note that more than one
 * instance of this class can be created with different IO addresses.
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#ifndef _DM6814_HH
#define _DM6814_HH

// Local includes
#include "types.hh"
#include "rhexio.hh"

// Constants ------------------------------------------------------

#define ENC_LSB         0   // Incremental Encoder LSB
#define ENC_MSB         1   // Incremental Encoder MSB
#define ENC_CLEAR       2   // Incremental Encoder Chip Clear Register
#define ENC_HOLD        2   // Incremental Encoder Chip Hold Address
#define ENC_DIO         2   // Incremental Encoder Chip DIO Register
#define ENC_STATUS      3   // Incremental Encoder Chip Status Register
#define ENC_MODE        3   // Incremental Encoder Chip Mode Register

#define ENC_TIMER_CLCK  12  // Timer/Counter 0 (Read/Write)
#define ENC_TIMER_CLCK1 13  // Timer/Counter 1 (Read/Write)
#define ENC_TIMER_CLCK2 14  // Timer/Counter 2 (Read/Write)
#define ENC_TIMER_CTRL  15  // Timer/Counter Control Word (Write)
#define ENC_IRQ         16  // IRQ Register (Read/Write)
#define ENC_IRQ_STATUS  17  // IRQ Status Register (Read)

// Flag Masks
#define ENC_IRQ_ENC1    0x01
#define ENC_IRQ_ENC2    0x02
#define ENC_IRQ_ENC3    0x04
#define ENC_IRQ_P14     0x08

#define ENC_REG_CLEAR   0
#define ENC_REG_HOLD    1
#define ENC_REG_DIO     2

// DM6814 Timer freq. in Hz (format should be long int!)
#define DM6814_FREQ    (8000000L)

// The dm6814 class ----------------------------------------------

class dm6814 {
 private:
  uint16  iobase;
  uint    irq;
  uint    mode[3];
  uint    IRQreg;

  uint16  timerInitial[3];

 public:
  dm6814( uint16 io, uint i );

  // Register selection for IO reads/writes
  void   setMode( uint chip, uint newmode );
  void   selectReg( uint chip, uint8 select );

  // Encoder functionality
  void   holdEncoder( uint chip );
  void   loadEncoder( uint chip, uint16 start );
  uint16 readEncoder( uint chip );
  void   clearEncoder( uint chip );
  void   enableEncoder( uint chip );
  void   disableEncoder( uint chip );
  void   clearEncoderIRQ( uint chip );
  void   enableEncoderIRQ( uint chip, bool enable );
  void   enableEncoderClear( uint chip, bool enable );


  // Digital IO functionality
  uint8  readDIO( uint chip );
  void   writeDIO( uint chip, uint8 data );
  void   setDIODir( uint chip, bool dir0, bool dir1 );

  // Timer functionality
  void   setupTimer( uint timer, uint mode, uint32 hertz );
  uint16 readTimer( uint timer );
  void   timerInfo( uint timer, uint *initial, uint *clockFreq );

  // Board interrupt functionality
  void   setIRQPolarity( bool polarity );
  void   setIRQShare( bool enable );
  void   clearBoardIRQ( void );
  void   enableBoardIRQ( bool enable );
  bool   boardIRQStatus( void );
  void   loadIRQReg( uint8 value );

};

inline void dm6814::setMode( uint chip, uint newmode ) {
  if ( newmode != mode[ chip ] ) {
    mode[ chip ] = newmode;
    routp( mode[ chip ], iobase + ENC_MODE + (chip << 2) );
  }
}

inline void dm6814::selectReg( uint chip, uint8 select ) {
  setMode( chip, ( mode[ chip ] & 0xFC ) | select );
}

inline void dm6814::holdEncoder( uint chip ) {
  selectReg( chip, ENC_REG_HOLD );
  routp( 0, iobase + ENC_HOLD + (chip << 2));
}

inline void dm6814::loadEncoder( uint chip, uint16 start ) {
  routpw( iobase + ENC_LSB + (chip << 2), start );
}

inline uint16 dm6814::readEncoder( uint chip ) {
  holdEncoder( chip );
  return uint16( rinpw( iobase + ENC_LSB + (chip << 2 ) ) );
}

inline void dm6814::clearEncoder( uint chip ) {
  selectReg( chip, ENC_REG_CLEAR );
  routp( 0, iobase + ENC_CLEAR + (chip << 2) );
}

inline void dm6814::enableEncoder( uint chip ) {
  setMode( chip, mode[ chip ] | 0x60 );
  routp( mode[ chip ], iobase + ENC_MODE + (chip << 2) );
}

inline void dm6814::disableEncoder( uint chip ) {
  setMode( chip, mode[ chip ] & 0x9F );
  routp( mode[ chip ], iobase + ENC_MODE + (chip << 2) );
}

inline void dm6814::clearEncoderIRQ( uint chip ) {
  selectReg( chip, ENC_REG_CLEAR );
  rinp( iobase + ENC_CLEAR + (chip << 2) );
}

inline void dm6814::enableEncoderIRQ( uint chip, bool enable ) {
  setMode( chip, ( mode[ chip] & 0xEF ) | (enable << 4) );
  routp( mode[ chip ], iobase + ENC_MODE + (chip << 2) );
}

// dm6814::enableEncoderClear : Enables or disables the external encoder clear signal
inline void dm6814::enableEncoderClear(uint chip, bool enable ) {
  setMode( chip, ( mode[ chip ] & 0x7F ) | ( enable << 7 ) );
  routp( mode[ chip ], iobase + ENC_MODE + (chip << 2) );
}

inline uint8 dm6814::readDIO( uint chip) {
  selectReg( chip, ENC_REG_DIO );
  return uint8( rinp( iobase + ENC_DIO + (chip << 2) ) );
}

inline void dm6814::writeDIO( uint chip, uint8 data ) {
  selectReg( chip, ENC_REG_DIO );
  routp( data, iobase + ENC_DIO + (chip << 2) );
}

inline void dm6814::setDIODir( uint chip, bool dir0, bool dir1) {
  //    Dir0: (P0.0)                0 = In  1 = Out
  //    Dir1: (P0.1)                0 = In  1 = Out
  setMode( chip, ( mode[ chip ] & 0xF3 ) | (dir0 << 2) | (dir1 << 3) );
  routp( mode[ chip ], iobase + ENC_MODE + (chip << 2) );
}

inline uint16 dm6814::readTimer( uint timer ) {
  uint16 msb, lsb;

  // Latch the desired counter
  routp( ( timer << 6 ), iobase + ENC_TIMER_CTRL );
    
  lsb = uint8(rinp( iobase + ENC_TIMER_CLCK + ( timer ) ));
  msb = uint8(rinp( iobase + ENC_TIMER_CLCK + ( timer ) ));

  return uint16( ( ( msb << 8) | lsb ) );
}

inline void dm6814::clearBoardIRQ( void ) {  
  rinp(iobase + ENC_IRQ);
}

inline bool dm6814::boardIRQStatus( void ) {
  return bool( rinp( iobase + ENC_IRQ_STATUS ) & 0xF );
}

inline void dm6814::loadIRQReg( uint8 value ) {
  IRQreg = value;
  routp( IRQreg, iobase + ENC_IRQ );
}

inline void dm6814::enableBoardIRQ( bool enable ) {
  IRQreg = ( IRQreg & 0xFE ) | enable;
  routp( IRQreg, iobase + ENC_IRQ );
}

inline void dm6814::setIRQPolarity( bool polarity ) {
  IRQreg = ( IRQreg & 0xFD ) | (polarity << 1);
  routp( IRQreg, iobase + ENC_IRQ );
}

inline void dm6814::setIRQShare( bool enable ) {
  IRQreg = ( IRQreg & 0xFB ) | ( enable << 2 );
  routp( IRQreg, iobase + ENC_IRQ );
}

#endif
