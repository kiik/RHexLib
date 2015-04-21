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
 * $Id: ExtProfiler.cc,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Created       : Robert Peters, 05/29/2001
 * Last Modified : Robert Peters, 05/29/2001
 *
 ********************************************************************/

#include "ExtProfiler.hh"

ExtProfiler::ExtProfiler( void ) {

  x_points = NULL;
  y_points = NULL;
  begin_slopes = NULL;
  end_slopes = NULL;
  a = NULL;
  b = NULL;
  c = NULL;
  d = NULL;

  numpoints = 0;
  size = 0;

  startsize = 128;  // Default size to create an array
}

float ExtProfiler::getVal ( float x )
{
  if ( numpoints < 1 )
    MMFatalError("ExtProfiler::getVal", "Profile not initialized");

  // if x is before the first point
  if (x < x_points[0]) return y_points[0];

  // if x is after the last point
  if (x > x_points[numpoints - 1]) return y_points[numpoints - 1];

  // if x is in between the first and last points
  // finds the segment (x_points[i-1], x_points[i]]
  // excluding the first point in the segment, and including the last point
  for (i = 1; x > x_points[i]; i++);

  return (a[i]*x*x*x + b[i]*x*x + c[i]*x + d[i]);

}
float ExtProfiler::getSlope ( float x )
{
  if ( numpoints < 1 )
    MMFatalError("ExtProfiler::getSlope", "Profile not initialized");

  // if x is before the first point
  if (x < x_points[0]) return 0.0;

  // if x is after the last point
  if (x > x_points[numpoints - 1]) return 0;

  // if x is in between the first and last points
  // finds the segment (x_points[i-1], x_points[i]]
  // excluding the first point in the segment, and including the last point
  for (i = 1; x > x_points[i]; i++);

  return (3*a[i]*x*x + 2*b[i]*x + c[i]);
}
MotorTarget_t ExtProfiler::getTarget ( float x )
{
  if ( numpoints < 1 )
    MMFatalError("ExtProfiler::getTarget", "Profile not initialized");

  MotorTarget_t target;

  // if x is before the first point
  if (x < x_points[0])
  { 
    target.pos = a[0]*x*x*x + b[0]*x*x + c[0]*x + d[0];
    target.vel = 3*a[0]*x*x + 2*b[0]*x + c[0];
    target.acc = 6*a[0]*x + 2*b[0];
  }
  // if x is after the last point
  else if (x > x_points[numpoints - 1])
  {
    target.pos = y_points[numpoints - 1];
    target.vel = 0;
    target.acc = 0;
  }
  // if x is in between the first and last points
  else
  {
    // finds the segment (x_points[i-1], x_points[i]]
    // excluding the first point in the segment, and including the last point
    for (i = 1; x > x_points[i]; i++);

    target.pos = a[i]*x*x*x + b[i]*x*x + c[i]*x + d[i];
    target.vel = 3*a[i]*x*x + 2*b[i]*x + c[i];
    target.acc = 6*a[i]*x + 2*b[i];
  }

  return target;
}


void ExtProfiler::addSegment ( float x, float y, float begin_slope, float end_slope )
{
  float x1;
  float y1;
  float temp1;
  float temp2;
  float temp3;
  float temp4;

  // Creates the arrays if they're empty
  if (size < 1)
  {
    size = startsize;
    x_points = new float[size];
    y_points = new float[size];
    begin_slopes = new float[size];
    end_slopes = new float[size];
    a = new float[size];
    b = new float[size];
    c = new float[size];
    d = new float[size];
  }

  // Checks to see if the size of the array needs to be increased
  if ( numpoints + 1 > size )
  {
    size *= 2;

    // Allocate space
    float *temp_x_points = new float[size];
    float *temp_y_points = new float[size];
    float *temp_begin_slopes = new float[size];
    float *temp_end_slopes = new float[size];
    float *temp_a = new float[size];
    float *temp_b = new float[size];
    float *temp_c = new float[size];
    float *temp_d = new float[size];

    // Copy values
    for (i = 0; i < numpoints; i++)
    {
      temp_x_points[i] = x_points[i];
      temp_y_points[i] = y_points[i];
      temp_begin_slopes[i] = begin_slopes[i];
      temp_end_slopes[i] = end_slopes[i];
      temp_a[i] = a[i];
      temp_b[i] = b[i];
      temp_c[i] = c[i];
      temp_d[i] = d[i];
    }

    // Deallocate arrays
    delete[] x_points;
    delete[] y_points;
    delete[] begin_slopes;
    delete[] end_slopes;
    delete[] a;
    delete[] b;
    delete[] c;
    delete[] d; 
  }

  // Records the values
  x_points[numpoints] = x;
  y_points[numpoints] = y;
  begin_slopes[numpoints] = begin_slope;
  end_slopes[numpoints] = end_slope;

  if (numpoints > 0)
  {
    // sets variables taht are used more than once to simplify calculation
    x1 = x_points[numpoints - 1];
    y1 = y_points[numpoints - 1];
    temp1 = (x1 - x);
    temp2 = temp1 * temp1 * temp1;
    temp3 = (x1 * x - 2 * x1 * x1 + x * x);
    temp4 = (x1 * x1 + x1 * x - 2 * x * x);

    // Solving these equations: M = {{ x1^3, x1^2, x1, 1}, {x2^3,
    // x2^2, x2, 1}, {3 x1^2, 2x1, 1, 0}, {3 x2^2, 2x2, 1, 0}};
    // Simplify[(Inverse[M].{y1, y2, m1, m2})]

    a[numpoints] 
	  = ((begin_slope + end_slope) * temp1 - 2 * y1 + 2 * y) / temp2;
    b[numpoints] = (end_slope * temp3
                    - begin_slope * temp4
                    + 3 * (x1 + x) * (y1 - y)) / temp2;
    c[numpoints] = (end_slope * x1 * temp4
                    - x * ( begin_slope * temp3 + 6 * x1 * (y1 - y) )) / temp2;
    d[numpoints] = (x1 * x * (-end_slope * x1 * temp1 - begin_slope * x * temp1
                              + 3 * x * y1 - 3 * x1 * y)
                    - x * x * x * y1 + x1 * x1 * x1 * y) / temp2;
    //  printf("x1=%f, y1=%f, m1=%f, m2=%f, a=%f, b=%f, c=%f, d=%f\n", x1, y1, begin_slope, end_slope, a[numpoints], b[numpoints], c[numpoints], d[numpoints]);
  }

  numpoints++;
}
void ExtProfiler::addSegment ( float x, float y, float end_slope )
{
  // if it's the first point, use any value for the slopes
  if (numpoints < 1) addSegment( x, y, 0.0, end_slope );
  // Otherwise the the end slope from the last segment as the begin slope
  else addSegment( x, y, end_slopes[numpoints - 1], end_slope );
}
void ExtProfiler::addSegment ( float x, float y )
{
  // if it's the first point, use any value for the slopes
  if (numpoints < 1) addSegment( x, y, 0.0, 0.0 );
  // Otherwise the the slopes by figuring out the slope from the last
  // point to this new point
  else addSegment( x, y, 
                   (y - y_points[numpoints - 1]) 
                     / (x - x_points[numpoints - 1]), 
                   (y - y_points[numpoints - 1]) 
                     / (x - x_points[numpoints - 1]) );
}



void ExtProfiler::setup( Floats set_x, Floats set_y, 
                         Floats set_begin_slope, Floats set_end_slope) {
  // Resets the profile
  reset();

  // Checks the lengths of each Floats
  errorCheck( set_x.getCount(), set_y.getCount(), 
              set_begin_slope.getCount(), set_end_slope.getCount() );

  // Loops through all values, adding the segments to the profile
  for (i = 0; i < set_x.getCount(); i++)
  {
    addSegment( set_x.get(i), set_y.get(i), 
                set_begin_slope.get(i), set_end_slope.get(i));
  }

}

void ExtProfiler::setup( Floats set_x, Floats set_y, Floats set_end_slope ) {

  // Resets the profile
  reset();

  errorCheck( set_x.getCount(), set_y.getCount(), 
              set_x.getCount(), set_end_slope.getCount() );

  for (i = 0; i < set_x.getCount(); i++)
  {
    addSegment( set_x.get(i), set_y.get(i), set_end_slope.get(i));
  }
}

void ExtProfiler::setup( Floats set_x, Floats set_y )
{
  reset();

  errorCheck(set_x.getCount(), set_y.getCount(), 
             set_x.getCount(), set_x.getCount());

  for (i = 0; i < set_x.getCount(); i++)
  {
    addSegment( set_x.get(i), set_y.get(i) );
  }
}

void ExtProfiler::reset()
{
  if (numpoints > 0)
  {
    // Deallocate arrays
    delete[] x_points;
    delete[] y_points;
    delete[] begin_slopes;
    delete[] end_slopes;
    delete[] a;
    delete[] b;
    delete[] c;
    delete[] d;
  }

  size = 0;
  numpoints = 0;
}

void ExtProfiler::setupLinear( float x1, float y1, float x2, float y2 )
{
  reset();
  
  addSegment(x1, y1);
  addSegment(x2, y2);
}

void ExtProfiler::errorCheck( int x, int y, int m1, int m2 )
{
  if ( x < 1 )
    MMFatalError("ExtProfiler::setup", 
                 "X has less than one value");

  if ( x != y ) 
  {
    MMFatalError("ExtProfiler::setup", 
                 "X and Y have different lengths");
  }
  if ( x != m1 ) 
  {
    MMFatalError("ExtProfiler::setup", 
                 "X and begin slopes have different lengths");
  }
  if ( x != m2 ) 
  {
    MMFatalError("ExtProfiler::setup", 
                 "X and end slopes have different lengths");
  }
}
