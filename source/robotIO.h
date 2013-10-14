//      robomotor.h
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


#ifndef RoboMotor_class
#define RoboMotor_class

class RoboMotor
{
	public:
		RoboMotor();
		virtual ~RoboMotor();
		int tem;
		void InitPort();
		void ClosePort();
		void turnleft(int Speed);
		void turnright(int Speed);
		void justleft(int Speed);
		void justright(int Speed);
		void motorsoff(int Speed);
		void gostraight(int Speed);
		void gobackward(int Speed);
		void stop();

			
	private:
		/* add your private declarations */
};

#endif /* ROBOMOTOR_H */ 
