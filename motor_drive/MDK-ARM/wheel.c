#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

float deltas[20][2] ={
{0,0},
{0.01,0},
{0.02,0},
{0.03,0},
{0.04,0},
{0.05,0},
{0.06,0},
{0.07,0},
{0.08,0},
{0.09,0},
{0.1,0},
{0.09,0},
{0.08,0},
{0.07,0},
{0.06,0},
{0.05,0},
{0.04,0},
{0.03,0},
{0.02,0},
{0.01,0}
};

float velocities[20][4];

typedef struct 
{
	uint8_t id;
	uint8_t radius;
	uint8_t wheel_velcoity;
	
}wheel;

wheel w[4];
void wheel_init(void)
{
	int i;
	for(i = 0;i < 4;i++)
	{
		w[i].id = i;
	}
}

void calculate_position(void)
{
	int i,j;
	float x,y;
	for(i = 0;i < 20;i++)
	{
		for(j = 0;j < 2;j++)
		{
			
				x = deltas[i][0];
			
		
				y = deltas[i][1];
		}
		
	}
}

void calculate_robot_velocity(float x, float y)
{
  int i,j;
	float time = 0.01;
	float velocity[3] = {x/time,y/time,0};
	int coupling_matrix[4][3] = {{-1,1,1},{1,1,1},{-1,1,-1},{1,1,-1}};//const vanera global banauney
	
	for(i = 0;i < 4;i++)
	{ 
		w[i].wheel_velcoity = 0;
		for(j = 0;j < 3;j++)
		{
	    w[i].wheel_velcoity += velocity[j] * coupling_matrix[i][j];
			
    }
	}
}



/*int calculate_robot_velocity(wheel* wheel,robot* robot)
{
	int i,j;
	
  for(i=0;i<20;i++)
	{
		
		
		
		for(j=0;j<2;j++)
		{
			
			r.robot_velocity[j] = deltas[i][j]/r.time;
			calculate_wheel_velocity();
		}
		
	}

}
void calculate_wheel_velocity(wheel* wheel, robot* robot)
{
	int k,l,m,n;
	int coupling_matrix[4][3] = {{-1,1,1},{1,1,1},{-1,1,-1},{1,1,-1}};
	for(k = 0;k < 4;k++)
	{
		for(l = 0;l < 3;l++)
		{
	    w[k].wheel_velcoity += r.robot_velocity[k] * coupling_matrix[k][l];
			
    }
	}

	
		for(n = 0; n < 4; n++)
		{
			velocities[i][n] = w[n].wheel_velcoity;
		}
		
	
}	*/

