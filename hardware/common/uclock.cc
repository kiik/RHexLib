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
 * $Id: uclock.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Uluc Saranli, 01/05/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

/*  int uclock( unsigned long seconds[2])
 *
 *  uclock() puts the current time into seconds[0] (seconds)
 *      and seconds[1] (micro-seconds)
 *  uclock() returns 1 on success, and 0 on failure
 *      (will only fail if read of 8254 chip fails)
 *
 *  NOTE: this module needs to be linked -T1 
 *
 *
 * This facility uses either one of two methods for accurate time measurement, 
 * based on what type of hardware is available.
 *
 *
 *  The 8254 timer 0 is used to trigger hardware interrupt 0 which Proc
 *  uses to keep time (update its cycle count).  We use Proc's cycle count
 *  as a base, add in Proc's start time to adjust to local time, and add the
 *  current cycle count of the 8254 timer 0.  By reading the 8254 directly,
 *  we can return times with a finer resolution than the ticksize
 *  (0.5 to 55 msec) which is by default set to 10msec.
 *
 *  uclock() assumes the 8254 timer 0 is set for mode 3 and is using
 *  2 byte binary count.  After reading the current count, we have to convert
 *  it to cycles consumed.
 *
 * Things to know while converting...
 *
 * 1) 8254 counts down so we have to subtract the current count
 *  from the re-load count to get cycles consumed.
 *
 * 2) mode 3 is a bit weird....
 *  it loads the count, raises OUT, and counts down by 2
 *  when it reaches 1, it reloads the count, lowers OUT
 *      and counts down by 2 again....
 *  therefore you need to divide the comsumed cycles by 2
 *  AND you need to check the OUT pin to determine
 *      if you are in the first sub-cycle or second sub-cycle
 *  when in the second sub-cycle, add half of the re-load count
 *
 * in other words: the 8254 will count down twice from the re-load count
 *  (by 2's) before trigger hardware interrupt
 * fisrt time down, OUT will be high
 * second time down, OUT will be low */

/*  NOTE: uclock() (based on 1,000,000 iterations) consumes around:    
 *          micro seconds   speed index machine
 *           6.0        20877       586/150MHz
 *           9.5        12508       586/90MHz
 *          15.5         5893       486/80MHz
 *          18.0         2448       486/33MHz
 *             287.0         2455       486/33MHz SX (ouch!) */

#ifdef _QNX4_

#include <i86.h>
#include <conio.h>
#include <math.h>
#include <sys/osinfo.h>
#include <sys/psinfo.h>
#include <stdio.h>

unsigned long cycle[2];
double freq;

// -------------------------------------------------------------------
// rdtsc() - Read Time Stamp Counter from a Pentium processor
//
// The 64 bit value is written into memory pointed to by ptr.
// 
void rdtsc64( unsigned long *ptr );
#pragma aux rdtsc64 = "db 0fh,31h" "mov [ebx],eax" "mov [ebx+4],edx" \
        parm nomemory [ebx] modify exact nomemory [eax edx];

// -------------------------------------------------------------------
// uclock()
// 
// Implements an accurate clock functionality via either the Pentium cycle
// counter or the motherboard timer.
//
typedef enum { UCLOCK_UNINIT, UCLOCK_TIMER, UCLOCK_PENTIUM } UclockMode;

int uclock( unsigned long *seconds ) {

  static struct _timesel volatile far *tp = 0;
  static long half_cycle_count;
  static UclockMode mode = UCLOCK_UNINIT;

  unsigned char status;
  long inter_count;
  long timer_count;
  double total_time;

  //    first time called, we need to do some initializing....
  if ( mode == UCLOCK_UNINIT ){
    struct _psinfo proc_data;
    struct _osinfo os_data;

    // Retrieve the OS info
    qnx_psinfo( 1, 1, &proc_data, 0, 0 );
    qnx_osinfo( 0, &os_data );

    // Retrieve the pointer to the time infor structure
    tp = (struct _timesel volatile far *) MK_FP( os_data.timesel, 0);
    half_cycle_count = tp->cnt8254 / 2;
    freq = tp->cycles_per_sec;

    // If the CPU we are using is a Pentium or higher, than we need to 
    // use the Pentium cycle counter for timing operations.
    if ( os_data.cpu >= 586 )
      mode = UCLOCK_PENTIUM;
    else
      mode = UCLOCK_TIMER;
  }

  if ( mode == UCLOCK_TIMER ) {
    // This mode looks at the intermediate timer count value for the timer which
    // the OS uses for its internal clock. Slow but accurate, this is our best
    // bet for non-Pentium class CPUs.

    // Load the maximum count for initialization.
    timer_count = tp->cnt8254;

    // We need to repeat this until we get a sufficiently large count to avoid
    // nasty timer wraparound problems
    while ( tp->cnt8254 - timer_count < 5 || timer_count < 5 ) {

      // read 8254 timer chip timer 0
      _disable();
      outp( 0x43, 0xC2 );             // latch status and count of counter 0
      cycle[0] = tp->cycle_lo;
      cycle[1] = tp->cycle_hi;
      status = inp( 0x40 );

      if ( !( status & 0x40 ) ) {     // check for valid count

        if ( status & 0x36 ) {        // check for: mode 3, 2byte binary count

          timer_count = inp( 0x40 );            // lsb
          timer_count |= ( inp( 0x40 ) << 8 );  // msb 
          _enable();
          inter_count = (tp->cnt8254 - timer_count) / 2;
          if ( !( status & 0x80) )    // if OUT pin low second sub-cycle
            inter_count += half_cycle_count;

        } else {   // bad mode - throw away data but do the 2 reads anyway

          _enable();
          inp( 0x40 );
          inp( 0x40 );
          return ( 0 );

        }
      } else{ 
        // bad status - do a bunch of reads to try to return sanity to the 8254

        _enable();

        inp( 0x40 );
        inp( 0x40 );
        inp( 0x40 );

        return ( 0 );
      }
    }
  } else if ( mode == UCLOCK_PENTIUM ) {
    // Yay!, Pentium cycle counter is available. In this case,
    // cycle[0] and cycle[1] will contain the cycle count from the
    // Pentium.

    inter_count = 0;
    rdtsc64( cycle ); // Read the cycle count.

  } else {
    // Invalid mode. This should not happen!

    return 1;

  }

  // convert cycle count to seconds
  // Combine cycle[1], cycle[1] and inter_count to a double.
  total_time = cycle[1];
  total_time = 
    (total_time * 0x10000) * 0x10000 
    + cycle[0]                      
    + inter_count;

  total_time *= 1/freq;  // Convert to seconds.

  // break (double)seconds to (u_long)seconds and (u_long)micro-seconds
  seconds[0] = total_time;
  seconds[1] = ( total_time - seconds[0] ) * 1000000L;

  return 1;
}

#endif // _QNX4_
