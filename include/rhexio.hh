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
 * $Id: rhexio.hh,v 1.2 2001/07/12 17:14:10 ulucs Exp $
 *
 * Macro wrappers for the low level io function calls. 
 *
 * Created       : Uluc Saranli, 12/21/2000
 * Last Modified : Uluc Saranli, 12/21/2000
 *
 ********************************************************************/

#ifndef _RHEXIO_HH
#define _RHEXIO_HH

#ifdef _LINUX_
#include <sys/io.h>
//#include <delay.h>

#define rinp( port ) ( inb( port ) )
#define rinpw( port ) ( inw( port ) )
#define routp( value, port) ( outb( value, port ) )
#define routpw( value, port) ( outw( value, port ) )

#define rdelay( usec ) usleep( usec )

#endif // #ifdef _LINUX_

#ifdef _QNX4_
#include <conio.h>
#include <i86.h>

#define rinp( port ) ( inp( port ) )
#define rinpw( port ) ( inpw( port ) )
#define routp( value, port ) ( outp( port, value ) )
#define routpw( value, port ) ( outpw( port, value ) )

#define rdelay( usec ) sleep( usec )

#endif // #ifdef _QNX4_

#endif // #ifndef _RHEXIO_HH
