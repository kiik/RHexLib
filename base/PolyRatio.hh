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
 * $Id: PolyRatio.hh,v 1.2 2001/07/12 17:14:09 ulucs Exp $
 *
 * Ratios of Polynomials representation class 
 *
 * Created       : Haldun Komsuoglu, 01/21/2001
 * Last Modified : Haldun Komsuoglu, 06/27/2001
 *
 ********************************************************************/

#ifndef _POLYRATIO_HH
#define _POLYRATIO_HH

#include "Polynomial.hh"

//---------------------------------------------------------------------
// The Ratio of Polynomials class -------------------------------------
//---------------------------------------------------------------------
class PolyRatio {

 private:

  Polynomial    Num;   // Numerator polynomial object
  Polynomial    Den;   // Denominator polynomial object

 public:

  // Default constructor
  PolyRatio( void ) { Num = 1; Den = 1;};
  // Constant polynomial constructor (conversion law) with a given
  // constant
  PolyRatio( double c) {Num = c; Den = 1;};
  // Buffer constructor with element value initialization
  PolyRatio( double N[], int orderN,
             double D[], int orderD) { Num = Polynomial(N, orderN);
                                   Den = Polynomial(D, orderD);};
  // Buffer constructor with element value initialization and power sign
  // assignment
  PolyRatio( double N[], int orderN,
             double D[], int orderD,
         int dPS)                { Num = Polynomial(N, orderN, dPS);
                                   Den = Polynomial(D, orderD, dPS);};



  // Sets the ratio of polynomial order sign interperation
  void setPowSign( int Sign ) { Num.setPowSign( Sign );
                                Den.setPowSign( Sign );};

  // Gets the polynomial order sign interperation
  int getPowSign( void ) { return Num.getPowSign(); };

  // Scales the coefficients of the numerator and denominator
  // so that the constant term of the denominator becomes unity
  void Unity_b0( void ) { double    s = 1/Den.getEntry(0);
                          Num *= s;
                          Den *= s; };

  // Returns the requested numerator coefficient
  double NumeratorCoeff( int index ) { return Num.getEntry(index); };

  // Returns the requested denominator coefficient
  double DenominatorCoeff( int index ) { return Den.getEntry(index); };

  // Returns the requested numerator coefficient
  double *NumCoeffListPntr( void ) { return Num.CoeffList(); };

  // Returns the requested denominator coefficient
  double *DenCoeffListPntr( void ) { return Den.CoeffList(); };

  // Evaluation at a given value, x
  double operator()( double x) { return (Num(x) / Den(x)); };


  // Ratio of Polynomial multiplication/assignment operation
  PolyRatio& operator*=( PolyRatio& PRequ ) { Num *= PRequ.Num;
                                              Den *= PRequ.Den;
                                              return *this;};

  // Polynomial addition/assiginment operation
  PolyRatio& operator+=( PolyRatio& PRadd ) { PolyRatio PR2;
                                              PR2 = PRadd;
                                              Num *= PR2.Den;
                                              PR2.Num *= Den;
                          Num += PR2.Num;
                          Den *= PR2.Den;
                                              return *this;}; 

  // Polynomial subtraction/assignment operation
  PolyRatio& operator-=( PolyRatio& PRdiff ) { PolyRatio PR2;
                                               PR2 = PRdiff;
                           Num *= PR2.Den;
                           PR2.Num *= Den;
                           Num -= PR2.Num;
                           Den *= PR2.Den;
                           return *this;};

  // Polynomial substitution/assignment operation
  PolyRatio& operator<<=( const PolyRatio& );

  // Displays (prints out) the polynomial in a formated form in the
  // standard output with the provided name
  void display( char* Pname ) {
    char str[128];
    sprintf( str, "%s_{num}(x)", Pname);
    Num.display( str );
    sprintf( str, "%s_{den}(x)", Pname);
    Den.display( str );
  };

  // Displays (prints out) the polynomial in a formated form in the
  // standard output with the default name
  void display( void ) { display("P"); };

};



#endif

