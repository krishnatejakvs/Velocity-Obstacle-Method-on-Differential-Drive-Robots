#include<stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;
#include <GL/glut.h>
#include<math.h>
#include<string.h>
//#define numberofobjects 3
float v=0.02,numberofobjects;
int condition=1;
float source_x;
float source_y;
float dest_x;
float dest_y;
void obs_init();
typedef struct objectInfo
{
	float vx;
	float vy;
	char  color;
	float xcent;
	float ycent;
	float radius;
	//	float slope;
}object;
object obj[10];

void display();
void idle();
//void check();
int update();
int	WIDTH = 500,i=0;
int	HEIGHT = 500;
int	FILLED = 1;
float	rotatetyre = 0.0;
float	movingfactorx = 0.01, movingfactory = 0.01, movingcirclex=-9.0, movingcircley = 0.0;
float	tmx = 0.0, tmy = 0.0;
float vl,vr;
double accuracy = 1;
int main( int argc, char **argv )
{
	//Initializations related to GLUT
	glutInit( &argc, argv );
	glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB );
	glutInitWindowSize( WIDTH, HEIGHT );
	glutCreateWindow( "Transformations in 2D" );
	printf("Enter robot coordinates:\n");
	scanf("%f %f",&source_x,&source_y);
	printf("Enter destination coordinates:\n");
	scanf("%f %f",&dest_x,&dest_y);
	printf("Enter number of obstacles:\n");
	scanf("%f",&numberofobjects);
	//Telling Glut about which function does what
	obs_init();
	glutDisplayFunc( display );
	glutIdleFunc( display );
	//	glutKeyboardFunc( keyboard );
	//	glutMouseFunc( mouse );
	//Setting the OpenGL Init State.
	//Following command decides how much part of window to be used for rendering.
	glViewport( 0, 0, WIDTH, HEIGHT );
	//For Time being assume this is Magic
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluOrtho2D ( -100, 100, -100, 100 );
	//Your transformation are controlled by the following Matrix
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	glutMainLoop();
	return 0;
}
float tan_inv(float num,float den)
{
	float th;
	if(num > 0 && den > 0)
	{
		th=atan(num/den);
	}
	if(num > 0 && den < 0)
	{
		th=atan(num/den)+M_PI;
	}
	if(num < 0 && den < 0)
	{
		th=atan(num/den)+M_PI;
	}
	if(num < 0 && den > 0)
	{
		th=2*M_PI+atan(num/den); 
	}
	if(num==0 && den!=0)
	{
		th=0;
	}
	if(num!=0 && den==0)
	{
		th=M_PI/2;
	}
	return th;
}
void obs_init()
{
//printf("dest1 = %f,angle1 = %f,angle = %f\n",dest1,angle1,angle);
	obj[0].xcent = source_x;  obj[0].ycent = source_y; obj[0].color = 'r'; obj[0].vx = 0; obj[0].radius = 5.0; obj[0].vy = 0;
	obj[1].xcent = 0;  obj[1].ycent = 0; obj[1].color = 'b'; obj[1].vx = -0.01;  obj[1].radius = 10.0; obj[1].vy = -0.01;
	obj[2].xcent = 40;  obj[2].ycent = 20; obj[2].color = 'b'; obj[2].vx = -0.03; obj[2].radius = 10; obj[2].vy = -0.06;
	obj[3].xcent = 30;  obj[3].ycent = 50; obj[3].color = 'b'; obj[3].vx = 0.05;  obj[3].radius = 10; obj[3].vy= -0.02;
	obj[4].xcent = 10;  obj[4].ycent = 30; obj[4].color = 'b'; obj[4].vx = 0.01;  obj[4].radius = 10; obj[4].vy= 0.04;
	obj[5].xcent = 50;  obj[5].ycent = 60; obj[5].color = 'b'; obj[5].vx = -0.05;  obj[5].radius = 10; obj[5].vy= 0.08;
	obj[6].xcent = 60;  obj[6].ycent = 20; obj[6].color = 'b'; obj[6].vx = 0.09;  obj[6].radius = 10; obj[6].vy= -0.04;
	obj[7].xcent = 20;  obj[7].ycent = 20; obj[7].color = 'b'; obj[7].vx = 0.05;  obj[7].radius = 10; obj[7].vy= -0.01;
	obj[8].xcent = 50;  obj[8].ycent = 20; obj[8].color = 'b'; obj[8].vx = -0.03;  obj[8].radius = 10; obj[8].vy= -0.01;
}
float headangle = 0;
float omnicx = obj[0].xcent+obj[0].radius*cos(headangle);
float omnicy = obj[0].ycent+obj[0].radius*sin(headangle);
void drawCircle(double radius,double startAngle,double endAngle,int t,double border,float r,float g,float b, double cx,double cy, char type)
{
	glPushMatrix();
	glTranslatef(cx,cy,0);
	glLineWidth(t);
	double th,xCircle,yCircle,j=startAngle;
	glColor3f(r,g,b);
	//Circle
	if(type=='f')
		glBegin(GL_POLYGON);
	else if(type=='l')
		glBegin(GL_LINE_STRIP);
	for(;j<=endAngle; j+=accuracy)
	{
		th=M_PI * j / 180.0;
		xCircle = radius * cos(th);
		yCircle = radius * sin(th);
		glVertex2f(xCircle, yCircle);
	}
	if(type=='f')
		glVertex2f(0,0);
	glEnd();
	glPopMatrix();
}
void right_left()
{
	vl=obj[0].vx*(cos(headangle)+sin(headangle))-obj[0].vy*(cos(headangle)-sin(headangle));
	vr=obj[0].vx*(cos(headangle)-sin(headangle))+obj[0].vy*(cos(headangle)+sin(headangle));
}
void robot_motion()
{
	float r,w,v;
	w=(vr-vl)/(2*obj[0].radius);
	v=(vl+vr)/2;

	obj[0].xcent = obj[0].xcent + (v/w)*sin(w+headangle) - (v/w)*sin(headangle);
	obj[0].ycent = obj[0].ycent + (v/w)*cos(headangle) - (v/w)*cos(w+headangle);

	omnicx = omnicx + obj[0].vx;
	omnicy = omnicy + obj[0].vy;


	headangle = headangle+w;

	//printf("r=%f V=%f heading angle=%f w=%f (x,y)=(%f,%f) (X,Y)=%f\n",r,V,headangle*180/M_PI,w,obj[0].xcent,obj[0].ycent,omnicx,omnicy);
	glColor3f(0.0f,0.0f,0.0f);
	drawCircle(obj[0].radius,0,360,2,0,0,0,1,obj[0].xcent,obj[0].ycent,'l');

	glColor3f(0.0f,0.0f,0.0f);
	drawCircle(2,0,360,2,0,0,0,1,obj[0].xcent-(obj[0].radius-2)*sin(headangle),obj[0].ycent+(obj[0].radius-2)*cos(headangle),'f');
	glColor3f(0.0f,0.0f,0.0f);
	drawCircle(2,0,360,2,0,0,0,1,obj[0].xcent+(obj[0].radius-2)*sin(headangle),obj[0].ycent-(obj[0].radius-2)*cos(headangle),'f');

	//	glColor3f(0.0f,0.0f,0.0f);
	//	drawCircle(omnicx,omnicy,2*obj[0].radius,'l');
}
int update(float dest)
{
//	printf("updating\n");
	int i,j,collision;
	float the_sum=dest,the_difference=dest,factor=M_PI/180,the_sum1,the_difference1,angle,angle1,temp;
	int h=1,condition=0,count_sum=0,state_sum=0,loop=1;
			while(1)
			{
			h=1;
			count_sum=0;
			while(count_sum<numberofobjects)
			{
			the_sum1=tan_inv((v*sin(the_sum)-obj[h].vy),(v*cos(the_sum)-obj[h].vx));
			if((the_sum1 < M_PI/2 && obj[h].ycent < obj[0].ycent))// && obj[h].xcent > obj[0].xcent)
			the_sum1=2*M_PI + the_sum1;
			if((the_sum1 > M_PI*3/2 && the_sum1 < 2*M_PI && obj[h].ycent > obj[0].ycent))//&& obj[h].xcent > obj[0].xcent)
			the_sum1 = the_sum1-2*M_PI;
			temp = sqrt((obj[0].xcent-obj[h].xcent)*(obj[0].xcent-obj[h].xcent)+(obj[0].ycent-obj[h].ycent)*(obj[0].ycent-obj[h].ycent));
			if(temp < obj[h].radius + obj[0].radius)
			{
				printf("collision occurred with object : %d\n",h);
			}
			angle = tan_inv((obj[h].ycent-obj[0].ycent),(obj[h].xcent-obj[0].xcent));
			angle1 = asin((obj[h].radius+obj[0].radius)/temp);
//			state_sum=0;
//			printf("angle=%f angle1=%f the_sum1=%f h=%d\n",angle*180/M_PI,angle1*180/M_PI,the_sum1*180/M_PI,h);
			if((the_sum1 >= angle+angle1 || the_sum1 <= angle-angle1))
			{
	//		printf("enter\n");
			count_sum++;
			h++;
//			state_sum = 1;
			obj[0].vx=v*cos(the_sum);
			obj[0].vy=v*sin(the_sum);
			}
			else
			break;
			}
			if(count_sum>=numberofobjects)
			return 1;
			
			int count_diff=0,state_diff=0;
			h=1;
			while(count_diff<numberofobjects)
			{
			the_difference1=tan_inv((v*sin(the_difference)-obj[h].vy),(v*cos(the_difference)-obj[h].vx));
			if((the_difference1 < M_PI/2 && obj[h].ycent < obj[0].ycent))// && obj[h].xcent > obj[0].xcent)
			the_difference1=2*M_PI + the_difference1;
			if((the_difference1 > M_PI*3/2 && the_difference1 < 2*M_PI && obj[h].ycent > obj[0].ycent))// && obj[h].xcent > obj[0].xcent)
			the_difference1= the_difference1-2*M_PI;
			temp = sqrt((obj[0].xcent-obj[h].xcent)*(obj[0].xcent-obj[h].xcent)+(obj[0].ycent-obj[h].ycent)*(obj[0].ycent-obj[h].ycent));
			if(temp < obj[h].radius + obj[0].radius)
			{
				printf("collision occurred with object : %d\n",h);
			}
			
			angle = tan_inv((obj[h].ycent-obj[0].ycent),(obj[h].xcent-obj[0].xcent));
			angle1 = asin((obj[h].radius+obj[0].radius)/temp);
//			printf("angle=%f angle1=%f the_difference1=%f h=%d\n",angle*180/M_PI,angle1*180/M_PI,the_difference1*180/M_PI,h);
			if((the_difference1 <= angle-angle1 || the_difference1 >= angle+angle1))
			{
			count_diff++;
			h++;
			obj[0].vx=v*cos(the_difference);
			obj[0].vy=v*sin(the_difference);
			
			}
			else
			break;
			}
			if(count_diff>=numberofobjects)
			return 1;
			
			if(count_sum<numberofobjects&&count_diff<numberofobjects)
			{
			the_sum=the_sum+M_PI/180;
			the_difference=the_difference-M_PI/180;
			loop++;
//			printf("angleupdate\n");
			}
			if(loop>180)
			{
			v=v+0.1;
//			count_sum=0;
//			count_diff=0;
//			h=1;
			}
			}
}
void obstacle_motion()
{
	float m,c,s;
	int nu=1;
	for(nu=1;nu<=numberofobjects;nu++)
	{
		if(obj[nu].xcent > 100-obj[nu].radius || obj[nu].xcent < -100+obj[nu].radius)
			obj[nu].vx = -obj[nu].vx;
		if(obj[nu].ycent > 100-obj[nu].radius || obj[nu].ycent < -100+obj[nu].radius)
			obj[nu].vy = -obj[nu].vy;
		drawCircle(obj[nu].radius,0,360,2,0,1,0,0,obj[nu].xcent,obj[nu].ycent,'f');
		obj[nu].xcent+=obj[nu].vx;
		obj[nu].ycent+=obj[nu].vy;
	}
}
void display( void )
{
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();
	float dest = tan_inv((dest_y-obj[0].ycent),(dest_x-obj[0].xcent));
	// Here I say what is the Background Color 
	// as in this color fills the whole window before you start drawing
	glClearColor( 255.0f, 255.0f, 255.0f, 0.0f );
	//Here it uses the above mentioned color to fill the window.
	glClear( GL_COLOR_BUFFER_BIT );
	//Here I start rendering my scene
	//My Axis
	
	glColor3f ( 0.0, 0.0, 0.0 );
	glBegin ( GL_LINE_LOOP );
	glVertex2f ( -90.5, -90.5 );
	glVertex2f ( -90.5, 90.5 );
	glVertex2f ( 90.5, 90.5 );
	glVertex2f ( 90.5, -90.5 );
	glEnd ();
	obstacle_motion();
//	robot_motion();
	if(condition==1)
	{
	float m,m1,a,b,c,temp,angle1,angle2,angle,dest,dest1;
	dest = tan_inv((dest_y-obj[0].ycent),(dest_x-obj[0].xcent));
	int i=1,j=1;
	j=update(dest);
	right_left();
	robot_motion();
//	obj[0].xcent+=obj[0].vx;
//	obj[0].ycent+=obj[0].vy;
///	drawCircle(obj[0].radius,0,360,2,0,0,0,1,obj[0].xcent,obj[0].ycent,'f');
//	if(obj[0].xcent==dest_x && obj[0].ycent==dest_y)
	if(abs(obj[0].xcent-dest_x) < 0.1 || abs(obj[0].ycent-dest_y) < 0.1)
	{
		obj[0].vx = 0;
		obj[0].vy = 0; 
//	drawCircle(obj[0].radius,0,360,2,0,0,0,1,obj[0].xcent,obj[0].ycent,'f');
		condition=0;
	}
	else
	{
	obj[0].vx=v*cos(dest);
	obj[0].vy=v*sin(dest);	
	}
	printf("robot is moving with v=%f vx= %f vy =%f\n",v,obj[0].vx,obj[0].vy);
	drawCircle(5,0,360,2,0,0,1,0,dest_x,dest_y,'f');
	
	}
	//This is when it appears on the screen
	glutSwapBuffers();
}
