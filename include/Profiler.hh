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
 * $Id: Profiler.hh,v 1.3 2001/07/18 21:33:08 ulucs Exp $
 *
 * Created       : Uluc Saranli, 12/4/2000
 * Last Modified : Uluc Saranli, 01/07/2000
 *
 ********************************************************************/

#ifndef _PROFILER_HH
#define _PROFILER_HH

#include <stdio.h>
#include "ModuleManager.hh"
#include "PositionControl.hh"

#ifndef M2_PI
#define M2_PI 6.2831854
#endif


// The CPG class can be used as a clock modulo a period. It's value is
// between 0 and 1, and is obtained using the method getVal(). It is
// essentially a sawtooth function of time. The period is set using
// the constructor or the reset() method.

class CPG : public Module {

  public:

    CPG ( int index ) 
      : Module ( "cpg", index, true, false ) { period = 1.0; }
    CPG ( char * n, int index ) 
      : Module ( n, index, true, false ) { period = 1.0; }
    CPG ( double p, int index ) 
      : Module ( "cpg", index, true, false ) { period = p; }
    CPG ( double p, char *n, int index ) 
      : Module ( n, index, true, false ) { period = p; }

    // the value of the CPG between 0 and 1.
    double getVal ( void ) { return val; };

    // module methods, not for public consumption
    void init ( void ) { val = 0.0; };
    void uninit ( void ) {};
    void activate ( void ) { mark = MMReadTime(); }
    void deactivate ( void ) {};

    inline void update ( void ) {

      double now = MMReadTime();
      double timepassed = now - mark;

      while ( timepassed > period ) {
         mark += period;
         timepassed = now - mark;
      }

      val = timepassed / period;
        
    }

    // reset the period and the initial value of the cpg
    void reset ( double p, double value ) {

      val = value;

      if ( value < 0 || value > 1 )
        mark = MMReadTime();
      else
        mark = MMReadTime() - value * p;

      period = p;

    }

  // Resets just the period
    void setPeriod( double p )
    {
      period = p;
      mark = MMReadTime() - val * p;
    }

  private: 

    double period, mark;
    double val;

};

//
// The Profiler class returns a piecewise linear function of time. Its
// parameters are a, b, start and stop. The function getVal() returns
// a if time<start and b if time>start and the point on the line from
// (start,a) to (stop,b) corresponding to the current time
// otherwise. Use for trajectory generation.
//
class Profiler {

  public:

    Profiler ( void ) {}

    // setup the parameters of the profiler, see description above.
    void setup ( double t1, double t2, double val1, double val2 ) {

      start = t1;
      stop = t2;
      a = val1;
      b = val2;

      vel = (b-a) / ( stop - start );  

    }

    // This returns the value of the profiler based on the current time
    MotorTarget_t getVal ( void ) {

      double now = MMReadTime();
      MotorTarget_t target;

      target.acc = 0;

      if ( now <= start ) {
        target.pos = a;
        target.vel = 0;
      } else if ( now >= stop ) {
        target.pos = b;
        target.vel = 0;
      } else {

        target.pos = vel * ( now - start ) + a;
        target.vel = vel;
      }

      return target;
    }

  private:

    double start, stop;  // start and stop times
    double a, b;         // values of f(start) and f(stop)
    double vel;          // slope of the line

};

//
// This class implements a cubic polynomial. It can be set up using
// the setup method with arguments x1, y1, m1, x2, y2 and m2 so that
// f(x1)=y1, f(x2)=y2 , f'(x1)=m1 and f'(x2)=m2. Used for trajectory
// generation by the CPGProfiler class.
//
class CubicFunction {

  public:

    CubicFunction ( void ) { a0 = a1 = a2 = a3 = 0.0; }

    // This sets up a cubic polynomial, f, so that f(x1)=y1, f(x2)=y2
    // , f'(x1)=m1 and f'(x2)=m2
    void setup ( double x1, double y1, double m1, 
                 double x2, double y2, double m2 ) {

      double dx = x1 - x2,
      dy = y1 - y2,
      denom = dx * dx * dx,
      temp1 =  x1 * ( x1 + x2 ) - 2*x2*x2,
      temp2 =  -2*x1*x1 + x2 * ( x1 + x2 );

      // check for divide by zero (i.e. if smooth_factor = 0)
      if ( dx == 0.0 ) dx = 1;

      a0 = ( x1 * ( x2 * ( -x1 * ( m2*dx + 3*y2 ) + x2 * ( 3*y1 - m1*dx ) ) 
                    + x1*x1*y2 ) - x2*x2*x2*y1 ) / denom;
      a1 = ( m2*x1 * temp1 - x2 * ( m1 * temp2 + 6*x1*dy ) ) / denom;
      a2 = ( -m1 * temp1 + m2 * temp2 + 3*(x1 + x2)*dy ) / denom;
      a3 = ( m1 * dx + m2*dx - 2*y1 + 2*y2 ) / denom;

      // printf ( "cubic setup: %lf, %lf, %lf, %lf, %lf, %lf\n", 
      //           x1, y1, m1, x2, y2, m2 );

    }
     
    // gets the value of the polynomial at x
    void valTar ( double x, MotorTarget_t * tar ) { 

      tar->pos = a0 + x * ( a1 + x * ( a2 + a3 * x ) );
      tar->vel = a1 + x * ( 2.0 * a2 + 3.0 * a3 * x );
      tar->acc = 2.0 * a2 + 6.0 * a3 * x;

    }

  private:

    double a0, a1, a2, a3;

};

//
// This class impliments a linear function. It can be set up using the
// setup method with arguments x1, y1, x2 and y2 so that f(x1)=y1,
// f(x2)=y2. Used for trajectory generation by the CPGProfiler class.
//
class LinearFunction {

  public:

    LinearFunction ( void ) { m = 1; b = 1; }

    // set the slope and y-intercept 
    void setm ( double slope ) { m = slope; }
    void setb ( double yint ) { b = yint; }

    // set up via two points that the line should go through
    void setup ( double x1, double y1, double x2, double y2 ) {

      if ( x2-x1 != 0.0 ) m=(y2-y1)/(x2-x1);
      else m = 1;
      b = y1 - m*x1;

    }
    
    // the value
    double val ( double x ) { return ( m*x + b ); }
    double slope ( void ) { return m; };

    // the inverse
    double inverse ( double y ) { 
      return ( ( y - b ) / m ); 
    }

    // the value and velocity
    void valTar ( double x, MotorTarget_t * tar ) { 

      tar->pos = m*x + b;
      tar->vel = m;
      tar->acc = 0.0;

    }

  private:

  double m, b;

};

//
// This class inplements RHex's characteristic motion profiler of fast
// slow fast. It essentially impliments a piecewise linear (or linear
// and cubic) function of the interval [0,1] (i.e. a cpg value. It has
// special functions to allow the profiler to change mid stride in the
// case of turning reverse of direction etc.
//
// Note: All angles are in RADIANS
//
class CPGProfiler {

  public:

    CPGProfiler ( void ) {
      mode = FORWARD;
      cubic = false;
    }

    // args: df = percentage of cpg period in stance (a.k.a. duty factor)
    //       sa = stance sweep angle, centered about offset,
    //       os = legg offset angle,
    //       sm = smoothing factor
    void initParams ( double df, double sa, double os, double sm ) {

      duty_factor = df;
      sweep_angle = sa;
      offset = os;
      if ( mode == BACKWARD ) p1 = -M_PI + os;
      else p1 = M_PI + os;
      p2 = os;
      smooth_factor = sm;
      setUpFunctions();

    }

    void setParams ( double df, double sa, double os ) {

      duty_factor = df;
      sweep_angle = sa;
      offset = os;
      setUpFunctions();

    }

    // whether or not to smooth out fast-slow and slow-fast
    // transitions with a cubic spline
    void setCubic ( bool c ) { cubic = c; }
    void setOffset ( double os ) { offset = os; }
    void setSmoothFactor ( double sm ) { smooth_factor = sm; }

    // Use this function to set the position of the leg motion
    // profiler upon begining the profie. This isn't always -Pi
    // because the offset may have changed due to turning or the user
    // may want the robot to switch diections.
    void setPosBegin ( void ) {
      p1 = f4.val(1.0); 
      if ( mode == BACKWARD ) // p1 should be less than 0
        while ( p1 > 0 ) p1 -= 2*M_PI; 
      else // p1 should be greater than 0
        while ( p1 < 0 ) p1 += 2*M_PI;  
      setUpFunctions();
    }

    // Similar to setPosBegin, except used in the middle of the
    // profile.
    void setPosHalfway ( void ) { 
      p2 = f2.val(0.5); 
      setUpFunctions();
    }

    // used to set up the profile after the parameters have changed,
    // probably should be private
    void setUpFunctions ( void );

    // change directions of the profiler for forward and backward
    // walking
    void setModeForward ( void ) { mode = FORWARD; }
    void setModeBackward ( void ) { mode = BACKWARD; }

    // The value as a function of [0,1]
    MotorTarget_t getVal ( double c );
    double getInverseVal ( double pos );

  private:

    // basic parameters;
    double duty_factor, sweep_angle, offset, smooth_factor;

    double x1, x2, y1, y2; // waypoints
    double p1, p2; // last values before next half cycles begin

    // linear parts of the piecewise linear profile
    LinearFunction f1, f2, f3, f4, g1, g2;

    // replacements for g1 and g2 if cubic pline is used
    CubicFunction g1c, g2c;

    enum { FORWARD, BACKWARD } mode;

    bool cubic;      // smooth using cubic splines?


};

#endif







