//      robomotor.cpp
//      
//      Copyright 2010 root <root@zer-MROBO>
//      
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.
#include "robomotor.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/io.h>
#include <sys/types.h>
#include <fcntl.h>
#define BASEPORT 0x378 /* lp1 */


RoboMotor::RoboMotor()
{
	
}
RoboMotor::~RoboMotor()
{
}
void RoboMotor::InitPort()
{
    if (ioperm(BASEPORT, 3, 1)) {perror("ioperm"); getchar(); exit(1);}
    tem = fcntl(0, F_GETFL, 0);
    fcntl (0, F_SETFL, (tem | O_NDELAY));
}
void RoboMotor::ClosePort()
{
    fcntl(0, F_SETFL, tem);
    outb(255, BASEPORT);
    if (ioperm(BASEPORT, 3, 0)) {perror("ioperm"); exit(1);}
}
void RoboMotor::turnleft(int Speed)
{
    outb(64, BASEPORT);
}
void RoboMotor::turnright(int Speed)
{
    outb(100, BASEPORT);
}
void RoboMotor::justleft(int Speed)
{
    outb(56, BASEPORT);
}
void RoboMotor::justright(int Speed)
{
    outb(101, BASEPORT);
}
void RoboMotor::motorsoff(int Speed)
{

}
void RoboMotor::gostraight(int Speed)
{
    outb(4, BASEPORT);
}
void RoboMotor::gobackward(int Speed)
{
    outb(32, BASEPORT);
}
void RoboMotor::stop()
{
    outb(255, BASEPORT);
}

