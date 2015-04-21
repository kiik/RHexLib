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
 * $Id: Filter.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Filter class
 *
 * Created       : Haldun Komsuoglu, 01/18/2001
 * Last Modified : Haldun Komsuoglu, 01/22/2001
 *
 ********************************************************************/

#ifndef _FILTER_HH
#define _FILTER_HH

#include <math.h>
#include "ModuleManager.hh"
#include "GenericSensor.hh" 

class DiscreteSystem;
class PolyRatio;
struct DiscreteSystemLinkedList;

//---------------------------------------------------------------------
// Module specific types ----------------------------------------------
//---------------------------------------------------------------------

// Filter types enum
typedef enum { RC, 
           BUTTERWORTH, 
           HAMMING, 
           HANNING,
           BOXCAR, 
           BLACKMAN } Filter_t;

// Filter parameters structure
struct FilterParam_t {

  Filter_t      Ftype;         // Type of the filter
  double        Fc;            // Cut-off frequency (Hz)
  double        Fs;            // Base sampling frequency (Hz)
  double        Gp;            // Passband gain
  int           numPoles;      // Number of poles for IIR, number of taps 
                               // for FIR
  double        parameter[10]; // Filter specific parameters
};


//---------------------------------------------------------------------
// The Filter class ---------------------------------------------------
//---------------------------------------------------------------------

class Filter : public Module {

 public:

  // Filter constructors ( designs the default unity gain amplifier filter )
  Filter( void );
  Filter( char *name );
  Filter( char *name, int index );

  // Common interface to design filters of various types
  int Design( FilterParam_t* );

  // Builds a filter with the user specified coefficients
  void UserDefinedFilter( double[], int, double[], int );

  // Connects the source to the filter input
  void AssignSource( GenericSensor *GivenSource ) { 
    source = GivenSource;
  };

  // Output value
  double output( void ) { return y; };
  
  // Module manager related procedures
  void init( void ) {};
  void uninit( void ) {};
  void activate( void );
  void deactivate( void ) {};
  void update( void );

 private:

  // ++++ Data fields
  DiscreteSystemLinkedList  *FilterHead; // Pointer to the head of the 
                                         // list of cascade sub-filter 
                                         // linked list
  DiscreteSystemLinkedList  *FilterLast; // Pointer to the last element 
                                         // of the list of cascade
                                         // sub-filter linked list
  
  double            Ts;          // Sampling period (sec)
  double            y;           // Buffer for output value  
  GenericSensor     *source;     // Pointer to the input signal 
                                 // source sensor 

  // ++++ Utility tools
  // Appends the given sub-filter discrete system to the end of the
  // linked list of sub-filters
  void AppendSubFilter( DiscreteSystem* );
  void ClearFilterList( void );

  // ++++ Accompanying math tools
  // Applies the bilinear transformation to the given s-domain transfer 
  // function and returns the resulting z-domain transfer function in 
  // the same ratio of polynomials variable
  int BilinearTransformation( PolyRatio * );
  // Adjusts the gain for a FIR filter 
  void AdjustGain( double[], int, double );
  // Sinc function generator
  void SincFunc( double[], int, double ); 
  // Computes the normalized frequency for a given absoulte physical
  // frequency in Hz
  double Omega_c( double Fc ) { return 2/Ts * tan( Fc * 2*Ts * M_PI  /2 ); };

  // ++++ Filter design procedures
  // RC IIR
  int Design_RC( FilterParam_t* );
  // BUTTERWORTH IIR
  int Design_BUTTERWORTH( FilterParam_t* );
  // BOXCAR FIR
  int Design_BOXCAR( FilterParam_t * );
  // HANNING FIR
  int Design_HANNING( FilterParam_t * );
  // HAMMING FIR
  int Design_HAMMING( FilterParam_t * );
  // BLACKMAN FIR
  int Design_BLACKMAN( FilterParam_t * );

};

#endif

