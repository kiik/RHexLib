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
 * $Id: vast.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * A 64 bit integer class library
 *
 * Created       : Uluc Saranli, 01/05/2001
 * Last Modified : Uluc Saranli, 06/27/2001
 *
 ********************************************************************/

// The classes vast and uvast implement signed and unsigned 64 bit
// integer data types, respectively.

#ifndef _VAST_HH
#define _VAST_HH

// ----------------------------------------------------------
// LINUX definitions ----------------------------------------

#ifdef _LINUX_
// In Linux, we have the long long data type

typedef long long vast;
typedef unsigned long long uvast;

#define VAST_TO_LONG( v ) long(v)
#define VAST_TO_DOUBLE( v ) double(v)

#endif // _LINUX_

// ---------------------------------------------------------
// QNX4 definitions ----------------------------------------

#ifdef _QNX4_
// QNX needs custom implementation
#include <sys/types.h>

typedef ulong_t l64[2];

//
// note: if you change these, you must rewrite the
// assembly code below
//
enum {
    _L64_LO = 0,
    _L64_HI = 1
};

typedef signed char l64stat_t;

extern l64stat_t  ul_add_diff(l64 dst, l64 hi, l64 lo);
extern l64stat_t  ul_add(l64 dst, const l64 src);
extern l64stat_t  ul_sub(l64 dst, const l64 src);
extern l64stat_t  ul_mul(l64 dst, const l64 src);
extern l64stat_t  ul_div(l64 dst, const l64 src);
extern l64stat_t  ul_mod(l64 dst, const l64 src);
extern l64stat_t  ul_divmod(l64 mod, l64 dst, l64 src);

extern l64stat_t  ul_addu(l64 dst, ulong_t val);
extern l64stat_t  ul_subu(l64 dst, ulong_t val);
extern l64stat_t  ul_mulu(l64 dst, ulong_t val);
extern l64stat_t  ul_divu(l64 dst, ulong_t val);
extern l64stat_t  ul_modu(l64 dst, ulong_t val);
extern l64stat_t  ul_divmodu(l64 mod, l64 dst, ulong_t val);


extern l64stat_t  long2ul(l64 dst, long val);
extern l64stat_t  ulong2ul(l64 dst, ulong_t val);
extern l64stat_t  ul2long(long *dst, l64 src);
extern l64stat_t  ul2ulong(ulong_t *dst, l64 src);
extern l64stat_t  ul2double(double *dst, l64 src);

extern void       ul_copy(l64 dst, l64 src);

extern l64stat_t  ul_cmp(const l64 arg1, const l64 arg2);
extern l64stat_t  ul_cmpu(const l64 arg1, const l64 arg2);
//-
// comparison operators:
//
#define ul_gt(x,y) (ul_cmp(x,y) > 0)
#define ul_ge(x,y) (ul_cmp(x,y) >= 0)
#define ul_eq(x,y) (ul_cmp(x,y) == 0)

#define ul_lt(x,y) ul_gt(y,x)
#define ul_le(x,y) ul_ge(y,x)
#define ul_ne(x,y) !ul_eq(x,y)

#define ul_gtu(x,y) (ul_cmpu(x,y) > 0)
#define ul_geu(x,y) (ul_cmpu(x,y) >= 0)
#define ul_equ(x,y) (ul_cmpu(x,y) == 0)

#define ul_ltu(x,y) ul_gtu(y,x)
#define ul_leu(x,y) ul_geu(y,x)
#define ul_neu(x,y) !ul_equ(x,y)

#define ul_make(d, h, l) ((d)[_L64_LO] = (l), (d)[_L64_HI] = (h))
#define ul_copy(a,b)   ((a)[0] = (b)[0], (a)[1] = (b)[1])


#pragma aux ul_add = \
    "mov    ebx, [edx]" \
    "add    [eax], ebx" \
    "mov    ebx, [edx+4]" \
    "adc    [eax+4], ebx" \
    "setc   al" \
    parm [eax] [edx] modify exact [ebx al] value [al];
#pragma aux ul_sub = \
    "mov    ebx, [edx]" \
    "sub    [eax], ebx" \
    "mov    ebx, [edx+4]" \
    "sbb    [eax+4], ebx" \
    "setc   al" \
    parm [eax] [edx] modify exact [ebx al] value [al];

#pragma aux ul_add_diff \
 parm [ebx] [esi] [edi] \
 modify exact [eax edx] \
 value [al] \
 = \
    "mov eax, [esi]" \
    "sub eax, [edi]" \
    "mov edx, 4[esi]" \
    "sbb edx, 4[edi]" \
    "add [ebx], eax" \
    "adc 4[ebx], edx" \
    "setc al" \

#pragma aux ul_mul = \
    "fild   qword ptr [eax]" \
    "fild   qword ptr [edx]" \
    "fmul" \
    "fistp  qword ptr [eax]" \
    parm [eax] [edx] modify exact [8087 al] value [al];

#pragma aux ul2double = \
    "fild   qword ptr [edx]" \
    "fstp   qword ptr [eax]" \
    parm [eax] [edx] modify exact [8087 al] value [al];

#pragma aux ul_addu = \
    "add    [eax], edx" \
    "adc    [eax+4], 0" \
    "setc   al" \
    parm [eax] [edx] modify exact [al] value [al];
#pragma aux ul_subu = \
    "sub    [eax], edx" \
    "sbb    [eax+4], 0" \
    "setc   al" \
    parm [eax] [edx] modify exact [al] value [al];

//-
// notice the arguments for this forces the long value onto the stack.
// The x87 can't access registers directly.
//
#pragma aux ul_mulu = \
    "fild   qword ptr [eax]" \
    "fild   dword ptr [esp]" \
    "fmul" \
    "fistp qword ptr [eax]" \
    parm [eax] modify exact [8087 al] value [al];



#pragma aux ul_cmp = \
    "mov    ebx, [edx]" \
    "sub    ebx, [eax]" \
    "jne    noteq" \
    "mov    ebx, [edx+4]" \
    "sbb    ebx, [eax+4]" \
    "setl   al" \
    "jle    done" \
    "dec    al" \
    "jmp    done" \
    "noteq: mov ebx, [edx+4]" \
    "sbb    ebx, [eax+4]" \
    "setl   al" \
    "jl    done" \
    "dec    al" \
    "done: " \
    parm [eax] [edx] modify exact [ebx al] value [al];

#pragma aux ul_cmpu = \
    "mov    ebx, [edx]" \
    "sub    ebx, [eax]" \
    "jne    noteq" \
    "mov    ebx, [edx+4]" \
    "sbb    ebx, [eax+4]" \
    "setb   al" \
    "jbe    done" \
    "dec    al" \
    "jmp    done" \
    "noteq: mov ebx, [edx+4]" \
    "sbb    ebx, [eax+4]" \
    "setb   al" \
    "jb    done" \
    "dec    al" \
    "done: " \
    parm [eax] [edx] modify exact [ebx al] value [al];

/*#pragma aux ul_cmp = \
    "mov    ebx, [eax+4]" \
    "cmp    ebx, [edx+4]" \
    "ja     bigger" \
    "jb     smaller" \
    "mov    ebx, [eax]" \
    "cmp    ebx, [edx]" \
    "ja bigger" \
    "smaller: sete  al" \
    "dec    al" \
    "jmp    done" \
    "bigger: mov al,1" \
    "done:" \
    parm [eax] [edx] modify [ebx eax] value [al];*/



//-
// conversion functions
//

#pragma aux long2ul = \
    "cdq" \
    "mov    [ebx], eax" \
    "mov    [ebx+4], edx" \
    "setc   al" \
    parm [ebx] [eax] modify exact [eax edx] value [al];

#pragma aux ulong2ul = \
    "mov    [edx], eax" \
    "xor    eax, eax" \
    "mov    [edx+4], eax" \
    parm [edx] [eax] modify exact [eax] value [al];

#pragma aux ul2long = \
    "push   dword ptr [edx]" \
    "pop    dword ptr [eax]" \
    "mov    eax, [edx+4]" \
    "and    eax, eax" \
    "setc   al" \
    parm [eax] [edx] modify exact [eax] value [al];

#pragma aux ul2ulong = \
    "push   dword ptr [edx]" \
    "pop    dword ptr [eax]" \
    "mov    eax, [edx+4]" \
    "and    eax, eax" \
    "setc   al" \
    parm [eax] [edx] modify exact [eax] value [eax];


// uvast Class: Stores signed 64 bit integers -----------------------
class vast {
  friend class uvast;

 private:

  l64   value;

 public:

  vast( void ) { long2ul( value, 0 ); };
  vast( long i ) { long2ul( value, i ); };
  vast( const vast& v ) { ul_copy( value, v.value ); };

  vast& operator=( const vast& v ) { ul_copy( value, v.value ); return *this; };
  vast& operator=( long i ) { long2ul( value, i ); return *this; };

  double getDouble( void ) { double res; ul2double( &res, value ); return res; };
  long getLong() { long i; ul2long( &i, value); return i; };

  vast &operator+=( const vast& a ) { ul_add( value, a.value ); return *this; };
  vast &operator-=( const vast& a ) { ul_sub( value, a.value ); return *this; };
  vast &operator*=( const vast& a ) { ul_mul( value, a.value ); return *this; };

  friend vast operator+( const vast& a, const vast& b );
  friend vast operator-( const vast& a );
  friend vast operator-( const vast& a, const vast& b );
  friend vast operator*( const vast& a, const vast& b );
  friend int operator>( const vast& a, const vast& b );
  friend int operator>=( const vast& a, const vast& b );
  friend int operator==( const vast& a, const vast& b );
  friend int operator<( const vast& a, const vast& b );
  friend int operator<=( const vast& a, const vast& b );
  friend int operator!=( const vast& a, const vast& b );

  //  void print( void ) { printf( "0x%08x%08x\n", value[1], value[0] ); };
};

inline vast operator +( const vast& a, const vast& b ) {
  vast result = a;
  ul_add( result.value, b.value );
  return result;
}

inline vast operator -( const vast& a ) {
  vast result = 0;
  ul_sub( result.value, a.value );
  return result;
}

inline vast operator -( const vast& a, const vast& b ) {
  vast result = a;
  ul_sub( result.value, b.value );
  return result;
}

inline vast operator *( const vast& a, const vast& b ) {
  vast result = a;
  ul_mul( result.value, b.value );
  return result;
}

inline int operator>( const vast& a, const vast& b ) {
  return int( ul_gt( a.value, b.value) );
}

inline int operator>=( const vast& a, const vast& b ) {
  return int( ul_ge( a.value, b.value) );
}

inline int operator==( const vast& a, const vast& b ) {
  return int( ul_eq( a.value, b.value) );
}

inline int operator<( const vast& a, const vast& b ) {
  return int( ul_lt( a.value, b.value) );
}

inline int operator<=( const vast& a, const vast& b ) {
  return int( ul_le( a.value, b.value) );
}

inline int operator!=( const vast& a, const vast& b ) {
  return int( ul_ne( a.value, b.value) );
}

#define VAST_TO_LONG( v ) (v).getLong()
#define VAST_TO_DOUBLE( v ) (v).getDouble()

// uvast Class: Stores unsigned 64 bit integers -----------------------
class uvast {
  friend class vast;

 private:

  l64   value;

 public:

  uvast( void ) { ulong2ul( value, 0 ); };
  uvast( ulong_t i ) { ulong2ul( value, i ); };
  uvast( const uvast& v ) { ul_copy( value, v.value ); };
  uvast( const vast& v ) { ul_copy( value, v.value ); };

  uvast& operator=( const uvast& v ) { ul_copy( value, v.value ); return *this; };
  uvast& operator=( long i ) { ulong2ul( value, i ); return *this; };

  ulong_t getLong() { ulong_t i; ul2ulong( &i, value); return i; };

  uvast &operator+=( const uvast& a ) { ul_add( value, a.value ); return *this; };
  uvast &operator-=( const uvast& a ) { ul_sub( value, a.value ); return *this; };
  uvast &operator*=( const uvast& a ) { ul_mul( value, a.value ); return *this; };

  friend uvast operator+( const uvast& a, const uvast& b );
  friend uvast operator-( const uvast& a );
  friend uvast operator-( const uvast& a, const uvast& b );
  friend uvast operator*( const uvast& a, const uvast& b );
  friend int operator>( const uvast& a, const uvast& b );
  friend int operator>=( const uvast& a, const uvast& b );
  friend int operator==( const uvast& a, const uvast& b );
  friend int operator<( const uvast& a, const uvast& b );
  friend int operator<=( const uvast& a, const uvast& b );
  friend int operator!=( const uvast& a, const uvast& b );


  //  void print( void ) { printf( "0x%08x%08x\n", value[1], value[0] ); };
};

inline uvast operator +( const uvast& a, const uvast& b ) {
  uvast result = a;
  ul_add( result.value, b.value );
  return result;
}

inline uvast operator -( const uvast& a ) {
  uvast result = 0;
  ul_sub( result.value, a.value );
  return result;
}

inline uvast operator -( const uvast& a, const uvast& b ) {
  uvast result = a;
  ul_sub( result.value, b.value );
  return result;
}

inline uvast operator *( const uvast& a, const uvast& b ) {
  uvast result = a;
  ul_mul( result.value, b.value );
  return result;
}

inline int operator>( const uvast& a, const uvast& b ) {
  return int( ul_gtu( a.value, b.value) );
}

inline int operator>=( const uvast& a, const uvast& b ) {
  return int( ul_geu( a.value, b.value) );
}

inline int operator==( const uvast& a, const uvast& b ) {
  return int( ul_equ( a.value, b.value) );
}

inline int operator<( const uvast& a, const uvast& b ) {
  return int( ul_ltu( a.value, b.value) );
}

inline int operator<=( const uvast& a, const uvast& b ) {
  return int( ul_leu( a.value, b.value) );
}

inline int operator!=( const uvast& a, const uvast& b ) {
  return int( ul_neu( a.value, b.value) );
}
#endif // _QNX4_

#endif
