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
 * $Id: clock.cc,v 1.3 2001/08/08 22:54:32 mcmordie Exp $
 *
 * Michigan hardware implementation of the RHexLib clock facilities. 
 *
 * Created       : Uluc Saranli, 12/20/2000
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

#include "McGillHW.hh"

// The current implementation of the clock is as follows for different
// operating systems:
//
// QNX:
//   Currently, the low resolution system timer is used. A better solution
// will be implemented in the future.
//
// LINUX : 
//   Not implemented yet.

#include "ModuleManager.hh"
#include "rhexio.hh"
#include "io/uclock.hh"

#ifdef _QNX4_
#include <sys/osinfo.h>
#include <time.h>
static struct _timesel volatile far *tp = 0;
static struct _osinfo info;
static long startSec;
static long startNsec;
static CLOCK uclockDiff = 0;
#endif

void McGillHW::initClock( void ) {

#ifdef _QNX4_
  struct timespec res;
  CLOCK startClock;
#endif

#ifdef _QNX4_
  // Set the QNX time resolution to 10 ms
  res.tv_nsec = 1000000L;
  clock_setres( CLOCK_REALTIME, &res );

  // Retrieve the location of the operating system's time structures.
  // Note that this is VERY OS dependent, but it avoids a kernel call.
  // Normally, I would use clock_gettime() or clock(), but they are *HORRIBLY*
  // slow ( 55 microseconds on a 486DX100 !)
  qnx_osinfo( 0, &info );
  tp = (struct _timesel volatile far *) MK_FP( info.timesel, 0);

  // Read the current time as the base offset
  startSec = tp->seconds;
  startNsec = tp->nsec;

  // Wait for the cock tick to come
  startClock = readClock();
  while ( startClock == readClock() );

  // Start the uclock() facility for very accurate timing and
  // Figure out what the offset between the two clocks is
  uclockDiff = readUClock() - readClock() ;
#endif

}

CLOCK McGillHW::readClock( void ) {

  CLOCK clockval = 0;

#ifdef _QNX4_

  if ( tp != 0 ) {
    // Compute the clock value since the initialization
    clockval = 
      ( tp->seconds - startSec ) * 1000000
      + ( tp->nsec - startNsec ) / 1000;
  } else {
    clockval = 0;
  }

#endif

  return clockval;

}

CLOCK McGillHW::readUClock( void ) {
  unsigned long seconds[2];
  CLOCK clockval = 0;

#ifdef _QNX4_

  if ( !uclock( seconds ) )
    MMFatalError( "McGillHW::readUClock", "Cannot read uclock!" );

  clockval = CLOCK( seconds[0] ) * 1000000 + CLOCK( seconds[1] ) - uclockDiff;

#endif

  return clockval;
}
