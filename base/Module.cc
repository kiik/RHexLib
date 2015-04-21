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
 * $Id: Module.cc,v 1.3 2001/07/19 18:51:30 eklav Exp $
 *
 * Module class
 *
 * This file contains the implimentation of the Module class member functions 
 *
 * Created       : Uluc Saranli, 10/26/2000
 * Last Modified : Eric Klavins, 07/19/2001
 *
 ********************************************************************/

#include <stdio.h>
#include "ModuleManager.hh"

// Module::Module: Constructor for the Module class objects
Module::Module( char * n, int i, bool single, bool poll ) {

  state = UNINIT;
  usedby = NULL;
  grabCount = 0;
  name = n;
  index = i;
  single_user = single;
  polling = poll;
  next = prev = NULL;

}

// Module::addToList: A Linked List Utility Function
void Module::addToList( Module ** head ) {

  if ( (*head) == NULL ) {

    *head = this;
    next = prev = this;

  } else {

    Module *temp = *head;

    while ( temp->order < order && temp->next != *head )
      temp = temp->next;

    // This is necessary if the module is to be added at the end of the list
    if ( temp->order < order && temp->next == *head )
      temp = temp->next;

    next = temp;
    prev = temp->prev;
    temp->prev->next = this;
    temp->prev = this;

    if ( temp == *head && order <= temp->order )
      *head = this;

  }

}

// Module::removeFromList: A Linked List Utility Function
void Module::removeFromList( Module ** head ) {

  if ( prev == NULL || next == NULL ) {
    *head = NULL;
    return;
  }

  if ( this == (*head) ) {

    if ( next != *head )
      *head = next;
    else
      *head = NULL;

  } 

  prev->next = next;
  next->prev = prev;
  next = prev = NULL;

}
