// Tbug.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "math.h"
#include <iostream>
#include <conio.h>
#include <windows.h>
#include "CXBOXController.h"

using namespace std;
using namespace System;
using namespace System::IO::Ports;

SerialPort^ OpenSerialPort(System::String ^port, int baud);
void CloseSerialPorts(SerialPort ^Port);
void Sendpwm(int count, SerialPort ^robot);

CXBOXController* Player1;


float pi = (float)(3.14159265359);
int k;  								// Sub-Stepping Variable
float V[3];				// Position Vector for Center of Robot
float rot[3];			// Rotation Vector of Center of Robot

float x = V[0];							// Taken from Joystick
float y = V[1];							// Taken from Joystick
float z = V[2];							// Height of Robot body	
float n = rot[0];						// Rotation around x
float o = rot[1];						// Rotation around y
float a = rot[2];						// Rotation around z

// Leg Lengths
float l1 = 9.0;						// Center to frame 1 in x distance
float l2 = 4.5;							// Platform to Hip Frame in z
float l3 = 4.0;							// Hip to Knee distance
float l4 = 10.0;						// Knee to Wrist Distance
float l5 = 10.5;						// Wrist to foot vertical
float l6 = 2.0;							// Wrist to foot Horizontal

float t[4][4], t_inv[4][4];				// Initialize transform from center to foot frame
float D[6][3], Dtheta[6][3];			// Initialize D, Dtheta
float t1;
float t2;
float t3;							// Motor Angles	
float Dcurrent[6][3];					// Current Motor Angles
float tl;								// Leg Angle
float h_foot[6], h_foot_prev[6];				// Height of the foot
float delta_foot[6];						// Change in Foot height
float theta_cs[6][3];					// All current thetas

int kmax = 10;							// Max value of K before switching legs
int leg;								// Leg number
int legseries[3][2] = {					// Leg pattern
	{0, 3},
	{1, 4},
	{2, 5}
};

int s;									// leg series

float f_height;							// Foot Height
float f_height_total;

unsigned int PWM_val;					// PWM to send to motors

float vector_mag = 0.5;
float rot_mag = 1;

float max_angle[3] = {(float)(45.0*pi/180), (float)(60.0*pi/180), (float)(30.0*pi/180)};
float min_angle[3] = {(float)(-45.0*pi/180), (float)(-60.0*pi/180), (float)(-30.0*pi/180)};

int PWM[6][3];

float MAX[6][3] = {
	{1400.0,1850.0,1800.0},
	{1400.0,1875.0,1700.0},
	{2050.0,1950.0,1750.0},
	{1850.0,2000.0,1750.0},
	{1450.0,2050.0,1800.0},
	{1575.0,1950.0,1750.0}
};

float MIN[6][3] = {
	{825.0,650.0,1100.0},
	{850.0,650.0,1050.0},
	{1450.0,700.0,1150.0},
	{1250.0,700.0,1150.0},
	{950.0,800.0,1150.0},
	{950.0,700.0,1100.0}
};

float CENTER[6][3] = {
	{1100.0,1250.0,1400.0},
	{1100.0,1250.0,1300.0},
	{1800.0,1300.0,1400.0},
	{1550.0,1400.0,1400.0},
	{1250.0,1450.0,1450.0},
	{1250.0,1300.0,1350.0}
	
};

int PIN[6][3] = {
	{0,1,2},
	{17,18,19},
	{20,21,22},
	{23,24,25},
	{6,7,8},
	{4,3,5}
	
};

SerialPort^ OpenSerialPort(System::String ^port, int baud)
{
	SerialPort^ newport;
	try
	{
		newport = gcnew SerialPort(port, baud);
		newport->Open();
	}
    	catch (IO::IOException^ e  )
	{
		Console::WriteLine(e->GetType()->Name+": Port is not ready");
	}
	catch (ArgumentException^ e)
	{
		Console::WriteLine(e->GetType()->Name+": incorrect port name syntax, must start with COM/com");
	}
	return newport;
}

void CloseSerialPorts(SerialPort ^Port)
{
	Port->Close();
}

void Init_motors()
{
	for (int m = 0; m < 6; m++)
	{
		for (int n = 0; n < 3; n++)
		{
			D[m][n] = 0.0;
			Dtheta[m][n] = 0.0;
			Dcurrent[m][n] = 0.0;
		}
	}
}

void SendSinglepwm(int i, int j, int signal, SerialPort ^robot)
{
	robot->WriteLine("#"+ PIN[i][j] +"P" + signal + "T500\r");
	Sleep(500);
}
void thetalimits()
{
	int i;
	int j;

	for ( i = 0; i < 6; i++ )
	{
		for ( j = 0; j < 3; j++ )
		{
			if ( theta_cs[i][0] < (float)(-45.0*(pi/180.0)) )
			{
				theta_cs[i][0] = (float)(-45.0*(pi/180.0));
				Dcurrent[i][0] = (float)(-45.0*(pi/180.0));
			}
			else if ( theta_cs[i][0] > (float)(45*(pi/180.0)) )
			{
				theta_cs[i][0] = (float)(45.0*(pi/180.0));
				Dcurrent[i][0] = (float)(45.0*(pi/180.0));
			}
			if ( theta_cs[i][1] < (float)(-15.0*(pi/180.0)) )
			{
				theta_cs[i][1] = (float)(-15.0*(pi/180.0));
				Dcurrent[i][1] = (float)(-15.0*(pi/180.0));
			}
			else if ( theta_cs[i][1] > (float)(60.0*(pi/180.0)) )
			{
				theta_cs[i][1] = (float)(60.0*(pi/180.0));
				Dcurrent[i][1] = (float)(60.0*(pi/180.0));
			}
			if ( theta_cs[i][2] < (float)(-15.0*(pi/180.0)) )
			{
				theta_cs[i][2] = (float)(-15.0*(pi/180.0));
				Dcurrent[i][2] = (float)(-15.0*(pi/180.0));
			}
			else if ( theta_cs[i][2] > (float)(30.0*(pi/180.0)) )
			{
				theta_cs[i][2] = (float)(30.0*(pi/180.0));
				Dcurrent[i][2] = (float)(30.0*(pi/180.0));
			}
		}
	}
}
void conv_2_pwm()
{
	thetalimits();

	PWM[0][0] = (int)(366.06*theta_cs[0][0] + 1108.3);
	PWM[0][1] = (int)(572.96*theta_cs[0][1]  + 1250);
	PWM[0][2] = (int)(-668.45*theta_cs[0][2]  + 1433.3);
	PWM[1][0] = (int)(350.14*theta_cs[1][0]  + 1116.7);
	PWM[1][1] = (int)(584.89*theta_cs[1][1]  + 1258.3);
	PWM[1][2] = (int)(-620.7*theta_cs[1][2]  + 1350);
	PWM[2][0] = (int)(381.97*theta_cs[2][0]  + 1766.7);
	PWM[2][1] = (int)(596.83*theta_cs[2][1]  + 1316.7);
	PWM[2][2] = (int)(-572.96*theta_cs[2][2]  + 1433.3);
	PWM[3][0] = (int)(381.97*theta_cs[3][0]  + 1550);
	PWM[3][1] = (int)(620.7*theta_cs[3][1]  + 1366.7);
	PWM[3][2] = (int)(-572.96*theta_cs[3][2]  + 1433.3);
	PWM[4][0] = (int)( 318.31*theta_cs[4][0]  + 1216.7);
	PWM[4][1] = (int)(596.83*theta_cs[4][1]  + 1433.3);
	PWM[4][2] = (int)(-620.7*theta_cs[4][2]  + 1466.7);
	PWM[5][0] = (int)(397.89*theta_cs[5][0]  + 1258.3);
	PWM[5][1] = (int)(596.83*theta_cs[5][1]  + 1316.7);
	PWM[5][2] = (int)(-620.7*theta_cs[5][2]  + 1400);
	
}

void all_zero(SerialPort ^robot)
{
	cout << "MOVING LEGS TO HOME POSITION";
	int i;
	int j;
	for ( i = 0; i < 6; i++ ) 
	{
		// Move leg up
		theta_cs[i][1] = (float)(30.0*pi/180);
		Dcurrent[i][1] = (float)(30.0*pi/180);
		conv_2_pwm();
		SendSinglepwm(i,1, PWM[i][1], robot);
		
		// Move wrist to 15 degrees
		theta_cs[i][2] = (float)(15.0*pi/180);
		Dcurrent[i][2] = (float)(15.0*pi/180);
		conv_2_pwm();
		SendSinglepwm(i,2, PWM[i][2], robot);
		
		// Center Hip
		theta_cs[i][0] = (float)(0.0*pi/180);
		Dcurrent[i][0] = (float)(0.0*pi/180);
		conv_2_pwm();
		SendSinglepwm(i,0, PWM[i][0], robot);

		// Center Knee
		theta_cs[i][1] = (float)(0.0*pi/180);
		Dcurrent[i][1] = (float)(0.0*pi/180);
		conv_2_pwm();
		SendSinglepwm(i,1, PWM[i][1], robot);
		
		for ( j = 0; j < 3; j++ )
		{
			Dtheta[i][j] = 0.0;
			D[i][j] = 0.0;
			//Dcurrent[i][j] = 0.0;
		}
		cout << ". ";
		delta_foot[i] = 0.0;
		h_foot_prev[i] = 0.0;
		h_foot[i] = 0.0;
	}
	V[0] = 0.0;
	V[1] = 0.0;
	V[2] = 0.0;
	rot[0] = 0.0;
	rot[1] = 0.0;
	rot[2] = 0.0;
	k = 0;
}

void Initialize(SerialPort ^robot)
{
	cout << "INITIALIZING" << endl;
	all_zero(robot);
	k = 0;
	leg = 0;
	V[0] = 0.0;
	V[1] = 0.0;
	V[2] = 0.0;
	rot[0] = 0.0;
	rot[1] = 0.0;
	rot[2] = 0.0;
	int i;
	for ( i = 0; i < 6; i++ ) 
	{
			Dcurrent[i][0] = 0.0;
			Dcurrent[i][1] = 0.0;
			Dcurrent[i][2] = (float)(15.0*pi/180);
	}
	cout << endl;
}

void Sendpwm(SerialPort ^robot)
{
	int i;
	int j;
	
		for ( i = 0; i < 6; i++ )
		{
			for ( j = 0; j < 3; j++ )
			{
				if ( PWM[i][j] > (int)MAX[i][j] )
				{
					PWM[i][j] = (int)MAX[i][j];
					cout << "Leg: " << i << " Joint: " << j << "passed MAX" << endl;
				}
				else if ( PWM[i][j] < (int)MIN[i][j] )
				{
					PWM[i][j] = (int)MIN[i][j];
					cout << "Leg: " << i << " Joint: " << j << "passed MIN" << endl;
				}
			}
		}

		robot->WriteLine("#0P" + PWM[0][0] + "s1000\r #1P" + PWM[0][1] + "s1000\r #2P" + PWM[0][2] + "s1000\r #3P" + PWM[5][1] + "s1000\r #4P" + PWM[5][0] + "s1000\r #5P" + PWM[5][2] + "s1000\r #6P" + PWM[4][0] + "s1000\r #7P" + PWM[4][1] + "s1000\r #8P" + PWM[4][2] + "s1000\r #23P" + PWM[3][0] + "s1000\r #24P" + PWM[3][1] + "s1000\r #25P" + PWM[3][2] + "s1000\r #20P" + PWM[2][0] + "s1000\r #21P" + PWM[2][1] + "s1000\r #22P" + PWM[2][2] + "s1000\r #17P" + PWM[1][0] + "s1000\r #18P" + PWM[1][1] + "s1000\r #19P" + PWM[1][2] + "s1000\r");
		//Sleep(50);
}

void Get_Dz()
{

	// This function finds D
	D[leg][0] = x + a*(l3*sin(t1 + tl) + l1*sin(tl) + l4*sin(t1 + tl)*cos(t2) - sin(t1 + tl)*cos(t2)*(l6*cos(t3) - l5*sin(t3)) + sin(t1 + tl)*sin(t2)*(l5*cos(t3) + l6*sin(t3))) + o*(l2 + l5*cos(t2 + t3) + l6*sin(t2 + t3) - l4*sin(t2));
	D[leg][1] = y - n*(l2 + l5*cos(t2 + t3) + l6*sin(t2 + t3) - l4*sin(t2)) - a*(l3*cos(t1 + tl) + l1*cos(tl) + l4*cos(t1 + tl)*cos(t2) - cos(t1 + tl)*cos(t2)*(l6*cos(t3) - l5*sin(t3)) + cos(t1 + tl)*sin(t2)*(l5*cos(t3) + l6*sin(t3)));
	D[leg][2] = (float)delta_foot[leg];
}

void Get_D()
{

	// This function finds D
	D[leg][0] = x + a*(l3*sin(t1 + tl) + l1*sin(tl) + l4*sin(t1 + tl)*cos(t2) - sin(t1 + tl)*cos(t2)*(l6*cos(t3) - l5*sin(t3)) + sin(t1 + tl)*sin(t2)*(l5*cos(t3) + l6*sin(t3))) + o*(l2 + l5*cos(t2 + t3) + l6*sin(t2 + t3) - l4*sin(t2));
	D[leg][1] = y - n*(l2 + l5*cos(t2 + t3) + l6*sin(t2 + t3) - l4*sin(t2)) - a*(l3*cos(t1 + tl) + l1*cos(tl) + l4*cos(t1 + tl)*cos(t2) - cos(t1 + tl)*cos(t2)*(l6*cos(t3) - l5*sin(t3)) + cos(t1 + tl)*sin(t2)*(l5*cos(t3) + l6*sin(t3)));
	D[leg][2] = (float)delta_foot[leg] + z - n*(l3*sin(t1 + tl) + l1*sin(tl) + l4*sin(t1 + tl)*cos(t2) - sin(t1 + tl)*cos(t2)*(l6*cos(t3) - l5*sin(t3)) + sin(t1 + tl)*sin(t2)*(l5*cos(t3) + l6*sin(t3))) + o*(l3*cos(t1 + tl) + l1*cos(tl) + l4*cos(t1 + tl)*cos(t2) - cos(t1 + tl)*cos(t2)*(l6*cos(t3) - l5*sin(t3)) + cos(t1 + tl)*sin(t2)*(l5*cos(t3) + l6*sin(t3)));
}

void Get_Dtheta()
{
	// This function finds Dtheta
	Dtheta[leg][0] = D[leg][0]*(-sin(t1 + tl)/(l3 - l6*cos(t2 + t3) + l5*sin(t2 + t3) + l4*cos(t2))) + D[leg][1]*(cos(t1 + tl)/(l3 - l6*cos(t2 + t3) + l5*sin(t2 + t3) + l4*cos(t2)));
	Dtheta[leg][1] = D[leg][0]*((cos(t1 + tl)*(l6*cos(t2 + t3) - l5*sin(t2 + t3)))/(l4*l5*cos(t3) + l4*l6*sin(t3))) + D[leg][1]*((sin(t1 + tl)*(l6*cos(t2 + t3) - l5*sin(t2 + t3)))/(l4*l5*cos(t3) + l4*l6*sin(t3))) + D[leg][2]*((l5*cos(t2 + t3) + l6*sin(t2 + t3))/(l4*(l5*cos(t3) + l6*sin(t3))));
	Dtheta[leg][2] = D[leg][0]*((cos(t1 + tl)*(l5*sin(t2 + t3) - l6*cos(t2 + t3) + l4*cos(t2)))/(l4*(l5*cos(t3) + l6*sin(t3)))) + D[leg][1]*((sin(t1 + tl)*(l5*sin(t2 + t3) - l6*cos(t2 + t3) + l4*cos(t2)))/(l4*l5*cos(t3) + l4*l6*sin(t3))) + D[leg][2]*(-(l5*cos(t2 + t3) + l6*sin(t2 + t3) - l4*sin(t2))/(l4*(l5*cos(t3) + l6*sin(t3))));
}
void Update_Current_Dtheta()
{

	// Updates the current motor values
	Dcurrent[leg][0] = Dcurrent[leg][0] + Dtheta[leg][0];	// Hip Servo
	Dcurrent[leg][1] = Dcurrent[leg][1] + Dtheta[leg][1];	// Knee Servo
	Dcurrent[leg][2] = Dcurrent[leg][2] + Dtheta[leg][2];	// Wrist Servo
}

void Choose_Thetal()
{
	tl = (float)(leg*60.0*pi/180) + (float)(30.0*pi/180);
}

void Choose_Leg()
{
	switch ( leg ) 
	{
		case 0:
			leg = 3;
			Choose_Thetal();
			
			break;
		case 1:
			leg = 4;
			Choose_Thetal();
			break;
		case 2:
			leg = 5;
			Choose_Thetal();
			break;
		case 3:
			leg = 1;
			Choose_Thetal();
			break;
		case 4:
			leg = 2;
			Choose_Thetal();
			break;
		case 5:
			leg = 0;
			Choose_Thetal();
			break;
		default:
			leg = 0;
			Choose_Thetal();
			break;
	}
}

void Increment()
{
	if ( k <= kmax )
	{
		if ( V[0] >= 0.25*vector_mag || V[0] <= -0.25*vector_mag || V[1] <= -0.25*vector_mag || V[1] >= 0.25*vector_mag )
		{
			k = k+1;
		}
	}
	else
	{
		k = 0;
		s = s+1;
		if ( s > 2 )
		{
			s = 0;
		}
	}
}
void Z_function()
{
	// Defines the Z-height of the foot frame for any sub-step
	if ( k > kmax )
	{
		h_foot_prev[leg] = 0.0;
		h_foot[leg] = 0.0;
		delta_foot[leg] = 0.0;
	}
	//h_foot[leg] = 0;
	else
	{
	h_foot[leg] =  (float)(10*k - k*k)/5;
	delta_foot[leg] = h_foot[leg] - h_foot_prev[leg];
	
	h_foot_prev[leg] = h_foot[leg];
	}
	
}

void Find_Foot_Height()
{
	
	if ( k == 0 )
	{
		f_height = 0;
		f_height_total = 0;
	}
	else
	{
		f_height = l4*sin(Dtheta[leg][1]) + l5*sin(Dtheta[leg][2]+Dtheta[leg][1]);
		f_height_total = f_height_total + f_height;
	}


}

void zkinematics()
{
		
		//printf("K = %i\n",k);
		//printf("Leg Number: %i\n",leg);
		//printf("Vector: x = %f,y = %f,z = %f\n", x,y,z);
		//printf("Height of foot will be: %f\n",h_foot[leg]);
		//printf("Change in height of foot: %f\n",delta_foot[leg]);
		Choose_Thetal();

		
		//float Dcurrent_deg[3] = {Dcurrent[leg][0],Dcurrent[leg][1],Dcurrent[leg][2]};
		//printf("Previous theta: %f, %f, %f\n", Dcurrent_deg[0],Dcurrent_deg[1],Dcurrent_deg[2]);
		

		Get_Dz();
		Get_Dtheta();
		Update_Current_Dtheta();

		//printf("Current D: %f, %f, %f\n", D[leg][0],D[leg][1],D[leg][2]);

		//float Dtheta_deg[3] = {Dtheta[leg][0],Dtheta[leg][1],Dtheta[leg][2]};
		//printf("Change in Theta's: %f, %f, %f\n", Dtheta_deg[0],Dtheta_deg[1],Dtheta_deg[2]);
		
		//float Dncurrent_deg[3] = {Dcurrent[leg][0],Dcurrent[leg][1],Dcurrent[leg][2]};
		//printf("New theta Current: %f, %f, %f\n",Dncurrent_deg[0],Dncurrent_deg[1],Dncurrent_deg[2]);

		theta_cs[leg][0] = Dcurrent[leg][0];
		theta_cs[leg][1] = Dcurrent[leg][1];
		theta_cs[leg][2] = Dcurrent[leg][2];
		//Find_Foot_Height();
		//printf("Actual Change in Height of Foot: %f\n",f_height);
		//printf("Actual Height of Foot: %f\n\n\n",f_height_total);
		
}

void kinematics()
{
		
		//printf("K = %i\n",k);
		//printf("Leg Number: %i\n",leg);
		//printf("Vector: x = %f,y = %f,z = %f\n", x,y,z);
		//printf("Height of foot will be: %f\n",h_foot[leg]);
		//printf("Change in height of foot: %f\n",delta_foot[leg]);
		Choose_Thetal();

		
		//float Dcurrent_deg[3] = {Dcurrent[leg][0],Dcurrent[leg][1],Dcurrent[leg][2]};
		//printf("Previous theta: %f, %f, %f\n", Dcurrent_deg[0],Dcurrent_deg[1],Dcurrent_deg[2]);
		

		Get_D();
		Get_Dtheta();
		Update_Current_Dtheta();

		//printf("Current D: %f, %f, %f\n", D[leg][0],D[leg][1],D[leg][2]);

		//float Dtheta_deg[3] = {Dtheta[leg][0],Dtheta[leg][1],Dtheta[leg][2]};
		//printf("Change in Theta's: %f, %f, %f\n", Dtheta_deg[0],Dtheta_deg[1],Dtheta_deg[2]);
		
		//float Dncurrent_deg[3] = {Dcurrent[leg][0],Dcurrent[leg][1],Dcurrent[leg][2]};
		//printf("New theta Current: %f, %f, %f\n",Dncurrent_deg[0],Dncurrent_deg[1],Dncurrent_deg[2]);

		theta_cs[leg][0] = Dcurrent[leg][0];
		theta_cs[leg][1] = Dcurrent[leg][1];
		theta_cs[leg][2] = Dcurrent[leg][2];
		//Find_Foot_Height();
		//printf("Actual Change in Height of Foot: %f\n",f_height);
		//printf("Actual Height of Foot: %f\n\n\n",f_height_total);
		
}

void angle_check()
{
	int i;
	int j;
	for ( i = 0; i < 6; i++ ) 
	{
		for ( j = 0; j < 3; j++ )
		{
			if ( theta_cs[i][j] > max_angle[j] )
			{
				theta_cs[i][j] = max_angle[j];
			}
			else if ( theta_cs[i][j] < min_angle[j] )
			{
				theta_cs[i][j] = min_angle[j];
			}
			//cout << "Leg: " << i << " Joint: " << j << " Angle: " << theta_cs[i][j] << endl;
		}
	}
}


void set_thetas()
{
		t1 = Dcurrent[leg][0];
		t2 = Dcurrent[leg][1];
		t3 = Dcurrent[leg][2];
}

void leg_movement()
{
	x = V[0];
	y = V[1];
	z = V[2];
	n = rot[0];
	o = rot[1];
	a = rot[2];
	
	int i;
	for ( i = 0; i < 2; i++ )
	{
		leg = legseries[s][i];
		Choose_Thetal();
		set_thetas();
		Z_function();
		kinematics();
	}
	// Other leg conditions
	x = -V[0]/(float)1.65;
	y = -V[1]/(float)1.65;

	int s1;
	for ( s1 = 0; s1 < 3; s1++)
	{
		for ( i = 0; i < 2; i++ )
		{
			if ( s1 != s )
			{
				leg = legseries[s1][i];
				Choose_Thetal();
				set_thetas();
				kinematics();
			}
		}
	}
	//Choose_Thetal();
	angle_check();	
	// Increment Condition
	Increment();
	//V[2] = 0.0;
}

void xy_vector(int x, int y)
{
	// Gets vector from lx and ly
	if ( x > -32769 && x < -26213 )
	{
		V[0] = (float)-vector_mag;
	}
	else if ( x > -26214 && x < -19660 )
	{
		V[0] = (float)-0.866*vector_mag;
	}
	else if ( x > -19661 && x < -13107 )
	{
		V[0] = (float)-0.5*vector_mag;
	}	
	else if ( x > -13108 && x < -7000 )
	{
		V[0] = (float)-0.2588*vector_mag;
	}
	else if ( x > -7001 && x < 7001 )
	{
		V[0] = (float)0.0;
	}
	else if ( x > 7000 && x < 13108 )
	{
		V[0] = (float)0.2588*vector_mag;
	}
	else if ( x > 13107 && x < 19661 )
	{
		V[0] = (float)0.5*vector_mag;
	}
	else if ( x > 19660 && x < 26214 )
	{
		V[0] = (float)0.866*vector_mag;
	}
	else if ( x > 26213 && x < 32768 )
	{
		V[0] = (float)vector_mag;
	}
		else
	{
		V[0] = (float)0.0;
	}

	if ( y > -32769 && y < -26213 )
	{
		V[1] = (float)-vector_mag;
	}
	else if ( y > -26214 && y < -19660 )
	{
		V[1] = (float)-0.866*vector_mag;
	}
	else if ( y > -19661 && y < -13107 )
	{
		V[1] = (float)-0.5*vector_mag;
	}	
	else if ( y > -13108 && y < -7000 )
	{
		V[1] = (float)-0.2588*vector_mag;
	}
	else if ( y > -7001 && y <= 0 )
	{
		V[1] = (float)0.0;
	}
	else if ( y > 0 && y < 7001 )
	{
		V[1] = (float)0.0;
	}
	else if ( y > 7000 && y < 13108 )
	{
		V[1] = (float)0.2588*vector_mag;
	}
	else if ( y > 13107 && y < 19661 )
	{
		V[1] = (float)0.5*vector_mag;
	}
	else if ( y > 19660 && y < 26214 )
	{
		V[1] = (float)0.866*vector_mag;
	}
	else if ( y > 26213 && y < 32768 )
	{
		V[1] = (float)vector_mag;
	}
	else
	{
		V[1] = (float)0.0;
	}


}

void Get_Vector()
{
	int lx;
	int ly;
		
	if(Player1->IsConnected())
	{
		lx = Player1->GetState().Gamepad.sThumbLX;
		ly = Player1->GetState().Gamepad.sThumbLY;
	}

	xy_vector(lx, ly);
	//cout << "Vx: " << V[0] << " Vy: " << V[1] << " Lx: " << lx << " Ly: " << ly <<  endl;

}


void rot_vector(int x, int y)
{
	// Gets vector from lx and ly
	if ( x < -21845 )
	{
		rot[0] = (float)-0.01;
	}
	else if ( x > -21846 && x < -10923 )
	{
		rot[0] = (float)-0.005;
	}
	else if ( x > -10924 && x < 10924 )
	{
		rot[0] = (float)0.0;
	}	
	else if ( x > 10924 && x < 21846)
	{
		rot[0] = (float)0.005;
	}
	else if ( x > 21846 )
	{
		rot[0] = (float)0.01;
	}
	else
	{
		rot[0] = (float)0.0;
	}

	if ( y < -21845 )
	{
		rot[1] = (float)-0.01;
	}
	else if ( y > -21846 && y < -10923 )
	{
		rot[1] = (float)-0.005;
	}
	else if ( y > -10924 && y < 10924 )
	{
		rot[1] = (float)0.0;
	}	
	else if ( y > 10924 && y < 21846)
	{
		rot[1] = (float)0.005;
	}
	else if ( y > 21846 )
	{
		rot[1] = (float)0.01;
	}
	else
	{
		rot[1] = (float)0.0;
	}


}


void Get_Rot()
{
	int rx;
	int ry;
		
	if(Player1->IsConnected())
	{

		rx = Player1->GetState().Gamepad.sThumbRX;
		ry = Player1->GetState().Gamepad.sThumbRY;
	}

	rot_vector(rx, ry);
	//cout << "Rotx: " << rot[0] << " Roty: " << rot[1] << " Rx: " << rx << " Ry: " << ry <<  endl;
}

void test()
{
		//cout << "K = " << k << "\n";
		leg_movement();	
		//float x_loc[6];
		//float y_loc[6];
		//float z_loc[6];
		//int L = leg;

		//for (int m = 0; m < 6; m++)
		//{
		//	leg = m;
		//	Choose_Thetal();
		//	x_loc[m] = l3*cos(theta_cs[m][0] + tl) + l1*cos(tl) + l4*cos(theta_cs[m][0] + tl)*cos(theta_cs[m][1]) - cos(theta_cs[m][0] + tl)*cos(theta_cs[m][1])*(l6*cos(theta_cs[m][2]) - l5*sin(theta_cs[m][2])) + cos(theta_cs[m][0] + tl)*sin(theta_cs[m][1])*(l5*cos(theta_cs[m][2]) + l6*sin(theta_cs[m][2]));
		//	y_loc[m] = l3*sin(theta_cs[m][0] + tl) + l1*sin(tl) + l4*sin(theta_cs[m][0] + tl)*cos(theta_cs[m][1]) - sin(theta_cs[m][0] + tl)*cos(theta_cs[m][1])*(l6*cos(theta_cs[m][2]) - l5*sin(theta_cs[m][2])) + sin(theta_cs[m][0] + tl)*sin(theta_cs[m][1])*(l5*cos(theta_cs[m][2]) + l6*sin(theta_cs[m][2]));
		//	z_loc[m] = l4*sin(theta_cs[m][1]) - l5*cos(theta_cs[m][1] + theta_cs[m][2]) - l6*sin(theta_cs[m][1] + theta_cs[m][2]) - l2;
		//	cout << "Leg: " << m << ", X: " << x_loc[m] << ", Y: " << y_loc[m] << ", Z: " << z_loc[m] <<"\n";

			//for (int n = 0; n < 3; n++)
			//{
				//x = 0.1;
				//Z_function();
				//kinematics();
				//printf("leg: %i, motor: %i, angle: %f\n",m,n,Dcurrent[m][n]);
			//}
		//}
		//cout << endl << endl;
		//leg = L;
		//Choose_Thetal();
}



int main(array<System::String ^> ^args)
{
	SerialPort^ robot;
	System::String^ robotport = "COM1";
	int RBbaud = 115200;
	robot = OpenSerialPort(robotport, RBbaud);

	//robot = OpenSerialPort(robotport, RBbaud);
	Player1 = new CXBOXController(1);

	//
	
	Initialize(robot);
	//leg_movement();
	//conv_2_pwm();
	//Sendpwm(robot);
	while(1)
	{
	//Get Body Height (D-Pad)	
	if ( Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_A )
	{
		all_zero(robot);
		Sendpwm(robot);
	}
	if ( Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP )
	{
		V[2] = -0.25;
	}
	else if ( Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN )
	{
		V[2] = 0.25;
	}
	else
	{
		V[2] = 0.0;
	}

	Get_Vector();
	Get_Rot();
	//V[1] = 4.0;
	test();
	conv_2_pwm();
	Sendpwm(robot);
	Sleep(50);

	}
	
	


	CloseSerialPorts(robot);
	return 0;
}

