/*
 * isr.c:
 *	Wait for Interrupt test program - ISR method
 *
 *	How to test:
 *	  Use the SoC's pull-up and pull down resistors that are avalable
 *	on input pins. So compile & run this program (via sudo), then
 *	in another terminal:
 *		gpio mode 0 up
 *		gpio mode 0 down
 *	at which point it should trigger an interrupt. Toggle the pin
 *	up/down to generate more interrupts to test.
 *
 * Copyright (c) 2013 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <unistd.h>


// globalCounter:
//	Global variable to count interrupts
//	Should be declared volatile to make sure the compiler doesn't cache it.

static volatile int globalCounter [8] ;


/*
 * myInterrupt:
 *********************************************************************************
 */

void myInterrupt0 (void) { ++globalCounter [0] ; }
void myInterrupt1 (void) { ++globalCounter [1] ; }
void myInterrupt2 (void) { ++globalCounter [2] ; }
void myInterrupt3 (void) { ++globalCounter [3] ; }
/*
void myInterrupt4 (void) { ++globalCounter [4] ; }
void myInterrupt5 (void) { ++globalCounter [5] ; }
void myInterrupt6 (void) { ++globalCounter [6] ; }
void myInterrupt7 (void) { ++globalCounter [7] ; }
*/

/*
 *********************************************************************************
 * main
 *********************************************************************************
 */

int main (void)
{
  int gotOne, pin ;
  int myCounter [8] ;

  for (pin = 0 ; pin < 8 ; ++pin) 
    globalCounter [pin] = myCounter [pin] = 0 ;

  wiringPiSetup () ;
/*
  pinMode ( 22, INPUT);
  pinMode ( 23, INPUT);
  pinMode ( 21, INPUT);
 */ 
  pullUpDnControl (21, PUD_UP);
  pullUpDnControl (22, PUD_UP);
  pullUpDnControl (23, PUD_UP);
  
  wiringPiISR (21, INT_EDGE_FALLING, &myInterrupt3) ;
  wiringPiISR (22, INT_EDGE_FALLING, &myInterrupt1) ;
  wiringPiISR (23, INT_EDGE_FALLING, &myInterrupt2) ;
  for (;;)
  {
    gotOne = 0 ;
    printf ("Waiting ... ") ; fflush (stdout) ;

    for (;;)
    {
      for (pin = 0 ; pin < 8 ; ++pin)
      {
	if (globalCounter [pin] != myCounter [pin])
	{
	  printf (" Int on pin %d: Counter: %5d\n", pin, globalCounter [pin]) ;
	  myCounter [pin] = globalCounter [pin] ;
	  ++gotOne ;
	}
      }
      if (gotOne != 0)
	break ;
    }
  }
  return 0 ;
}
