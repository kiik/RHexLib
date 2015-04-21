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
 * $Id: Profiler.cc,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Eric Klavins, 01/07/2001
 * Last Modified : Eric Klavins, 06/27/2001
 *
 ********************************************************************/

#include <stdio.h>
#include "Profiler.hh"

// This sets up the six functions used in the profiler base on the
// parameters. f1 and f4 are linear functions and are the fast parts
// of the profile going. In forward mode without any turning or
// offsets, they go from -Pi to -astance/2 and from astance/2 to
// Pi. f2 and f3 are the slow part, going from -astance/2 to 0 and
// from 0 to astance/2. g1 and g2 (or g1c and g2c) smooth out the
// transitions from f1 to f2 and from f3 to f4 respectively.
void CPGProfiler::setUpFunctions ( void ) {

  x1 = 0.5*(1-duty_factor);
  x2 = 0.5*(1+duty_factor);

  if ( mode == BACKWARD ) {
    y1 = -0.5*sweep_angle + offset;
    y2 = 0.5*sweep_angle + offset;
  } else {
    y1 = 0.5*sweep_angle + offset;
    y2 = -0.5*sweep_angle + offset;
  }

  f1.setup ( 0, p1, x1, y1 );
  f2.setup ( x1, y1, 0.5, offset );
  f3.setup ( 0.5, p2, x2, y2 );

  if ( mode == BACKWARD )
    f4.setup ( x2, y2, 1, M_PI+offset );
  else
    f4.setup ( x2, y2, 1, -M_PI+offset );
  

  g1.setup ( x1 - 0.5*smooth_factor, f1.val ( x1 - 0.5*smooth_factor ), 
             x1 + 0.5*smooth_factor, f2.val ( x1 + 0.5*smooth_factor ) );

  g2.setup ( x2 - 0.5*smooth_factor, f3.val ( x2 - 0.5*smooth_factor ),
             x2 + 0.5*smooth_factor, f4.val ( x2 + 0.5*smooth_factor ) );

  MotorTarget_t tar1, tar2;

  f1.valTar ( x1 - 0.5*smooth_factor, &tar1 );
  f2.valTar ( x1 + 0.5*smooth_factor, &tar2 );
  g1c.setup ( x1 - 0.5*smooth_factor, tar1.pos, tar1.vel, 
              x1 + 0.5*smooth_factor, tar2.pos, tar2.vel );

  f3.valTar ( x2 - 0.5*smooth_factor, &tar1 );
  f4.valTar ( x2 + 0.5*smooth_factor, &tar2 );
  g2c.setup ( x2 - 0.5*smooth_factor, tar1.pos, tar1.vel, 
              x2 + 0.5*smooth_factor, tar2.pos, tar2.vel );

}

// This just computes the value of the approriate piece of the profile.
MotorTarget_t CPGProfiler::getVal ( double c ) {

  MotorTarget_t target;

  while ( c >= 1.0 ) c -= 1.0;
  while ( c < 0.0 ) c += 1.0;

  if      ( c < ( x1 - 0.5*smooth_factor ) ) f1.valTar ( c, &target ); 
  else if ( c < ( x1 + 0.5*smooth_factor ) ) cubic ? g1c.valTar ( c, &target ) : g1.valTar ( c, &target );
  else if ( c < 0.5 )                        f2.valTar ( c, &target ); 
  else if ( c < ( x2 - 0.5*smooth_factor ) ) f3.valTar ( c, &target ); 
  else if ( c < ( x2 + 0.5*smooth_factor ) ) cubic ? g2c.valTar ( c, &target ) : g2.valTar ( c, &target ); 
  else                                       f4.valTar ( c, &target ); 

  return target;

}

// This just computes the value of the approriate piece of the inverse profile.
double CPGProfiler::getInverseVal ( double pos ) {

  while ( pos < -M_PI ) pos += M2_PI;
  while ( pos > M_PI ) pos -= M2_PI;

  //  return 0;

  if ( mode == FORWARD ) {

    if      ( pos > f1.val ( x1 - 0.5*smooth_factor ) ) return f1.inverse ( pos );
    else if ( pos > f2.val ( x1 + 0.5*smooth_factor ) ) return g1.inverse ( pos );
    else if ( pos > 0 )                                 return f2.inverse ( pos );
    else if ( pos > f3.val ( x2 - 0.5*smooth_factor ) ) return f3.inverse ( pos );
    else if ( pos > f4.val ( x2 + 0.5*smooth_factor ) ) return g2.inverse ( pos );
    else                                                return f4.inverse ( pos );

  } else {

    if      ( pos < f1.val ( x1 - 0.5*smooth_factor ) ) return f1.inverse ( pos );
    else if ( pos < f2.val ( x1 + 0.5*smooth_factor ) ) return g1.inverse ( pos );
    else if ( pos < 0 )                                 return f2.inverse ( pos );
    else if ( pos < f3.val ( x2 - 0.5*smooth_factor ) ) return f3.inverse ( pos );
    else if ( pos < f4.val ( x2 + 0.5*smooth_factor ) ) return g2.inverse ( pos );
    else                                                return f4.inverse ( pos );

  }

}  




