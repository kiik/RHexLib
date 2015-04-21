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
 * $Id: ExtProfiler.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Created       : Robert Peters, 05/29/2001
 * Last Modified : Robert Peters, 05/29/2001
 *
 ********************************************************************/

/*
 * Usage:
 *
 * -There must be at least one point added before any get commands will work
 * -The slope values for the first point are ignored 
 *     (either from AddSegment, or setup)
 * -Indexing beyond the smallest or largest value of x will return 
 *      the largest's or smallest's values of y
 * -Calling setup will reset any stored data points
 * -The x values must be increasing when added from either setup or addPoints
 */

#ifndef _EXTPROFILER_HH
#define _EXTPROFILER_HH

// Local includes
#include "types.hh"
#include "MotorControl.hh"

class ExtProfiler {

public:
  
  ExtProfiler ( void );

  float getVal ( float x );            // Gets the value at point x
  float getSlope ( float x );          // Gets the slope at point x
  MotorTarget_t getTarget( float x );  // Gets the value and slope
                                       // at point x, and returns 
                                       // it in a MotorTarget_t format

  // Adds a segment to the end of the profile Sets up the most general
  // of segments, using (x,y) as the next point, and begin_slope and
  // end_slope as the beginning and ending slope for this specific
  // segment.
  void addSegment ( float x, float y, float begin_slope, float end_slope );

  // Sets up a smooth cubic.  Uses begin_slope from last segment's
  // end_slope
  void addSegment ( float x, float y, float end_slope ); 

  // Sets up a linear line segment to this next point
  void addSegment ( float x, float y );

  // setup calls reset and loops through addSegment using Floats to
  // pass the variables
  void setup ( Floats set_x, Floats set_y, 
               Floats set_begin_slope, Floats set_end_slope );
  void setup ( Floats set_x, Floats set_y, Floats set_end_slope );
  void setup ( Floats set_x, Floats set_y );

  // Sets up a single line from (x1,y1) to (x2,y2)
  void setupLinear ( float x1, float y1, float x2, float y2 );

  // Resets the profile
  void reset ();

  private:

  // Checks the lengths of 
  void errorCheck(int x, int y, int m1, int m2);

  // Array pointers for important info
  float *x_points,
        *y_points,
        *begin_slopes,
        *end_slopes,
        *a,
        *b,
        *c,
        *d;
 
  int i,             // Used as an iterator
      numpoints,     // number of points in profile
      size,          // size of the array
      startsize;     // initial size of the array

};

#endif
