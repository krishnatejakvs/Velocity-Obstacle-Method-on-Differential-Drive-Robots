#include "botConnector.h"
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
int condition=1;
float dest_x=0,dest_y=6000,omnicx,omnicy,v = 200,numberofobjects = 2,vleft,vright,v1 = 100,ra[3][6]; // 0 = heading angle , 1 = x coordinate , 2 = y coordinate , 3 = vx , 4 = vy , 5 = radius 
BotConnector::BotConnector(int argc, char **argv)
{
	Aria::init();
	parser=new ArArgumentParser(&argc, argv);
	parser->loadDefaultArguments();
	robotConnector=new ArRobotConnector(parser, &robot);
	ArSonarDevice sonar;
	robot.addRangeDevice(&sonar);
	gotoPoseAction = new ArActionGoto("goto");
	robot.addAction(gotoPoseAction,95);
}


/**
 * connect() take cares of all the things needed to connect to the bot.
 * @args: None
 * @return: Connection successfull or not.
 */
bool BotConnector::connect()
{
	bool success = true; //Let's be optimistic and not pesimistic.
	if(!robotConnector->connectRobot())
	{
		ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
		if(parser->checkHelpAndWarnUnparsed())
		{
			// -help not given
			Aria::logOptions();
			Aria::exit(1);
			success = false;
		}
	}
	else if (!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::shutdown();
		success = false;
	}

	return success;
}

/**
 * getReadings() returns readings for all the sonar present on the robot.
 * To get the relation between sonar number and direction see getAngles().
 * @args: None
 * @return: A std::vector<int> containing all the sonar readings.
 */
vector<int> BotConnector::getReadings()
{

	int count = robot.getNumSonar();
	vector<int> Z;

	for(int i=0; i<count;i++){
		int reading =  robot.getSonarRange(i);
		Z.push_back(reading);
		//        printf("sensor reading=%d\n",reading);
	}
	//    ArSensorReading *reading = robot.getSonarReading(0);
	//    unsigned int gr = reading->getRange();
	//    ArPose lPose = reading->getLocalPose(), pose=reading->getPose();
	//    double lx,ly;
	//    lx=pose.getX();
	//    ly=pose.getY();
	//    unsigned int dd = sqrt(lx*lx + ly*ly);
	//    printf("lx=%f, ly=%f, r=%u, calc=%u\n",lx,ly,gr, dd);
	return Z;
}

/**
 * getAngles() gives the angle for all the sonar present w.r.t the robot.
 * @args: None
 * @return: A std::vector<int> containing all the sonar angles.
 */
void  BotConnector::getAngles(int bot_number)
{
	const ArRobotParams *roboParams = robot.getRobotParams();
	ra[bot_number][0] = robot.getTh()*(M_PI/180);
	ra[bot_number][1] = robot.getX();
	ra[bot_number][2] = robot.getY()-(bot_number*1000);
	if(bot_number!=0)
	{
		ra[bot_number][3] = v1*cos(ra[bot_number][0]);
		ra[bot_number][4] = v1*sin(ra[bot_number][0]);
	}
	printf("bot_number = %d,robot angle=%f, robot pose=(%f, %f)\n",bot_number,robot.getTh(),ra[bot_number][1],ra[bot_number][2]);
	/* for(int i=0; i<count;i++)
	   {
	   int sonarDir = roboParams->getSonarTh(i)+round(robot.getTh());
	   angle.push_back(sonarDir);
	//        printf("angle[%d]=%d\t",i,sonarDir);
	}*/
	printf("\n");
	//return angle;
}


void BotConnector::moveRobot(double r, double th)
{
	printf("r = %.2lf,r");
	ArTime start;

	robot.runAsync(true);
	robot.enableMotors();

	//rotating robot
	//    robot.lock();
	//    robot.setDeltaHeading(th);
	//    robot.unlock();
	//    ArUtil::sleep(10000);

	if( th!=0 )
	{
		robot.lock();
		robot.setHeading(th+robot.getTh());
		robot.unlock();
		start.setToNow();
		while (1)
		{
			robot.lock();
			if (robot.isHeadingDone(1))
			{
				printf("directMotionExample: Finished turn\n");
				robot.unlock();
				break;
			}
			if (start.mSecSince() > 15000)
			{
				printf("directMotionExample: Turn timed out\n");
				robot.unlock();
				break;
			}
			robot.unlock();
			ArUtil::sleep(100);
		}
	}

	//moving robot
	if( r!=0 )
	{
		robot.lock();
		robot.move(r);
		robot.unlock();
		start.setToNow();
		while (1)
		{
			robot.lock();
			if (robot.isMoveDone())
			{
				printf("directMotionExample: Finished distance\n");
				robot.unlock();
				break;
			}
			if (start.mSecSince() > 15000)
			{
				printf("directMotionExample: Distance timed out\n");
				robot.unlock();
				break;
			}
			robot.unlock();
			ArUtil::sleep(50);
		}
	}


	printf("Movement Done\n");
}

/**
 * setRobotVelocity() sets the robot's right and left wheel velocity for a specified time.
 * @args:
 * 	double vr - Velocity of right wheel in Milimeter/sec. Default = 200.
 * 	double vl - Velocity of left wheel in Milimeter/sec. Default = 200.
 * 	unsigned int time - Time in MiliSecond for which the robot is to move. Default = 2000.
 * @return: None.
 */
void BotConnector::setRobotVelocity_idle(double vr, double vl)
{
	robot.runAsync(true);
	robot.enableMotors();

	printf("Setting vl=%.2lf mm/sec, vr=%.2lf mm/sec\n", vl, vr);
	robot.lock();
	robot.setVel2(vr, vl);
	robot.unlock();
}
void BotConnector::setRobotVelocity(double vr, double vl, unsigned int time)
{
	robot.runAsync(true);
	robot.enableMotors();

	printf("Setting vl=%.2lf mm/sec, vr=%.2lf mm/sec, then sleeping for %d seconds\n", vl, vr, time/1000);
	robot.lock();
	robot.setVel2(vr, vl);
	robot.unlock();

	//Just to let you know that in Aria TIME exists!!
	//ArTime start;
	//start.setToNow();
	//while (start.secSince()<2)
	//{
	//    ArUtil::sleep(50);
	//}


	ArUtil::sleep(time);//argument passed is in MilliSecond.
	robot.lock();
	robot.stop();
	robot.unlock();
}

/**
 * disconnect() stops the robot and severs all the connections to it. Like Father disowns the son in hindi movies.
 * @args: None
 * @return: A std::vector<int> containing all the sonar angles.
 */
int BotConnector::disconnect()
{
	robot.stopRunning();
	robot.waitForRunExit();
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
void right_left()
{
	vleft=ra[0][3]*(cos(ra[0][0])+sin(ra[0][0]))-ra[0][4]*(cos(ra[0][0])-sin(ra[0][0]));
	vright=ra[0][3]*(cos(ra[0][0])-sin(ra[0][0]))+ra[0][4]*(cos(ra[0][0])+sin(ra[0][0]));
}
/*void robot_motion()
{
	float r,w,vc;
	w=(vright-vleft)/(2*ra[0][5]);
	vc=(vleft+vright)/2;
//	ra[0][1] = ra[0][1] + (vc/w)*sin(w+ra[0][0]) - (vc/w)*sin(ra[0][0]);
//	ra[0][2] = ra[0][2] + (vc/w)*cos(ra[0][0]) - (vc/w)*cos(w+ra[0][0]);
	ra[0][0] = ra[0][0]+w;
}*/
int update(float dest)
{
		printf("updating\n");
	int i,j,collision;
	float the_sum=dest,the_difference=dest,factor=M_PI/180,the_sum1,the_difference1,angle,angle1,temp;
	int h=1,count_sum=0,state_sum=0,loop=1;
	while(1)
	{
		h=1;
		count_sum=0;
		while(count_sum<numberofobjects)
		{
			the_sum1=tan_inv((v*sin(the_sum)-ra[h][4]),(v*cos(the_sum)-ra[h][3]));
			if((the_sum1 < M_PI/2 && ra[h][2] < omnicy))// && obj[h].xcent > obj[0].xcent)
				the_sum1=2*M_PI + the_sum1;
			if((the_sum1 > M_PI*3/2 && the_sum1 < 2*M_PI && ra[h][2] > omnicy))//&& obj[h].xcent > obj[0].xcent)
				the_sum1= the_sum1-2*M_PI;
			temp = sqrt((omnicx-ra[h][1])*(omnicx-ra[h][1])+(omnicy-ra[h][2])*(omnicy-ra[h][2]));
			if(temp < ra[h][5] + 2*ra[0][5])
			{
				printf("collision occurred with object sum : %d\n",h);
			}
			angle = tan_inv((ra[h][2]-omnicy),(ra[h][1]-omnicx));
			angle1 = asin((ra[h][5]+2*ra[0][5])/temp);
			//			state_sum=0;
			//			printf("angle=%f angle1=%f the_sum1=%f h=%d\n",angle*180/M_PI,angle1*180/M_PI,the_sum1*180/M_PI,h);
			if((the_sum1 >= angle+angle1 || the_sum1 <= angle-angle1))
			{
						printf("enter\n");
				count_sum++;
				h++;
				//			state_sum = 1;
				ra[0][3]=v*cos(the_sum);
				ra[0][4]=v*sin(the_sum);
			}
			else
				break;
		}
		if(count_sum>=numberofobjects)
			{
			printf("fucksum\n");
			return 1;
			}

		int count_diff=0,state_diff=0;
		h=1;
		while(count_diff<numberofobjects)
		{
			the_difference1=tan_inv((v*sin(the_difference)-ra[h][4]),(v*cos(the_difference)-ra[h][3]));
			if((the_difference1 < M_PI/2 && ra[h][2] <omnicy))// && obj[h].xcent > obj[0].xcent)
				the_difference1=2*M_PI + the_difference1;
			if((the_difference1 > M_PI*3/2 && the_difference1 < 2*M_PI && ra[h][2] > omnicy))// && obj[h].xcent > obj[0].xcent)
				the_difference1= the_difference1-2*M_PI;
			temp = sqrt((omnicx-ra[h][1])*(omnicx-ra[h][1])+(omnicy-ra[h][2])*(omnicy-ra[h][2]));
			if(temp < ra[h][5] + 2*ra[0][5])
			{
				printf("collision occurred with object diff: %d\n",h);
			}

			angle = tan_inv((ra[h][2]-omnicy),(ra[h][1]-omnicx));
			angle1 = asin((ra[h][5]+2*ra[0][5])/temp);
			//			printf("angle=%f angle1=%f the_difference1=%f h=%d\n",angle*180/M_PI,angle1*180/M_PI,the_difference1*180/M_PI,h);
			if((the_difference1 <= angle-angle1 || the_difference1 >= angle+angle1))
			{
				printf("enterdiff %d\n",h);				
				count_diff++;
				h++;
				ra[0][3]=v*cos(the_difference);
				ra[0][4]=v*sin(the_difference);

			}
			else
				break;
		}
		if(count_diff>=numberofobjects)
			{
			printf("fuckdiff\n");
			return 1;
			}

		if(count_sum<numberofobjects&&count_diff<numberofobjects)
		{
			the_sum=the_sum+M_PI/180;
			the_difference=the_difference-M_PI/180;
			loop++;
						printf("angleupdate\n");
		}
		if(loop>180)
		{
			v=v+100;
			printf("fucked\n");
			loop =1;
			the_sum = dest;
			the_difference = dest;
			//			count_sum=0;
			//			count_diff=0;
			//			h=1;
		}
	}
}

int main(int argc, char **argv)
{
	dest_x=0;
	dest_y=6000;
	v = 200;	
	numberofobjects = 2;
	v1 = 100;
	ra[0][5] = 400;
	ra[1][5] = 400;
	ra[2][5] = 400; 
	ra[3][5] = 400;
	//argv[1]="-rh";
	//argv[2]="10.2.36.7";
	//argc=3;
	BotConnector bc(argc, argv);
	if( bc.connect() )
	{
		printf( "Connected to Robot\n" );
	}
	else
	{
		printf( "Connection attempt to Robot failed, Sorry :(\n" );
		return EXIT_SUCCESS;
	}
	argv[1]="-rrtp";
	argv[2]="8102";
	argc=3;
	BotConnector bc1(argc, argv);
	if( bc1.connect() )
	{
		printf( "Connected to Robot1\n" );
	}
	else
	{
		printf( "Connection attempt to Robot failed, Sorry :(\n" );
		return EXIT_SUCCESS;
	}
//	bc.setRobotVelocity_idle(100,100); // initialising vr and vl
	bc.setRobotVelocity(100,100, 10000);
	bc.getAngles(0);// updating ra[][].
//	omnicx = ra[0][1]+ra[0][5]*cos(ra[0][0]);
//	omnicy = ra[0][2]+ra[0][5]*sin(ra[0][0]);
	//bc1.moveRobot(0,90);
//bc1.setRobotVelocity(100,100, 10000);
bc1.moveRobot(6000,90);
//bc1.setRobotVelocity(100,100, 30000);
bc1.moveRobot(0,180);
	argv[1]="-rrtp";
	argv[2]="8103";	
	argc=3;
	BotConnector bc2(argc, argv);
	if( bc2.connect() )
	{
		printf( "Connected to Robot2\n" );
	}
	else
	{
		printf( "Connection attempt to Robot failed, Sorry :(\n" );
		return EXIT_SUCCESS;
	}
bc2.moveRobot(2000,90);
//bc2.setRobotVelocity(100,100, 20000);
bc2.moveRobot(2000,-45);
//bc2.setRobotVelocity(200,200, 25000);
bc2.moveRobot(0,135);
/*	argv[1]="-rrtp";
  argv[2]="8104";
  argc=3;
  BotConnector bc3(argc, argv);
  if( bc3.connect() )
  {
          printf( "Connected to Robot\n" );
  }
  else
  {
          printf( "Connection attempt to Robot failed, Sorry :(\n" );
          return EXIT_SUCCESS;
  }
//bc.setVelocity1(-100,-100, 10000);
bc3.moveRobot(0,90);
bc3.setRobotVelocity(150,150, 30000);
bc3.moveRobot(0,-125);
bc3.setRobotVelocity(100,100, 20000);
bc3.moveRobot(0,180);*/
/*argv[1]="-rrtp";
  argv[2]="8105";
  argc=3;
  BotConnector bc4(argc, argv);
  if( bc4.connect() )
  {
          printf( "Connected to Robot\n" );
  }
  else
  {
          printf( "Connection attempt to Robot failed, Sorry :(\n" );
          return EXIT_SUCCESS;
  }
//bc.setVelocity1(-100,-100, 10000);
bc4.getAngles(4);
bc4.moveRobot(0,90);
bc4.setRobotVelocity(100,100, 40000);
bc4.moveRobot(0,0);
bc4.setRobotVelocity(200,200, 25000);
bc4.moveRobot(0,180);
argv[1]="-rrtp";
  argv[2]="8106";
  argc=3;
  BotConnector bc5(argc, argv);
  if( bc5.connect() )
  {
          printf( "Connected to Robot\n" );
  }
  else
  {
          printf( "Connection attempt to Robot failed, Sorry :(\n" );
          return EXIT_SUCCESS;
  }
//bc.setVelocity1(-100,-100, 10000);
bc5.getAngles(5);
bc5.moveRobot(0,90);
bc5.setRobotVelocity(200,200, 25000);
bc5.moveRobot(0,-80);
bc5.setRobotVelocity(300,300, 15000);
bc5.moveRobot(0,180);*/
bc.setRobotVelocity(-100,-100, 10000);
bc.moveRobot(0,90);
bc.getAngles(0);
bc1.getAngles(1);
bc2.getAngles(2);
omnicx = ra[0][1];
omnicy = ra[0][2]+300;
//bc3.getAngles(3);
/*bc4.getAngles(4);
bc5.getAngles(5);*/
 ArUtil::sleep(5000);
	
	int f;
	for(f=0;f<200;f++)
	{
			float dest,dest1;
			int j;
			dest = tan_inv((dest_y-omnicy),(dest_x-omnicx));
			j=update(dest);
			right_left();
		//	robot_motion();
 			bc.setRobotVelocity_idle(vright,vleft);
         		 bc1.setRobotVelocity_idle(100,100);
	  		bc2.setRobotVelocity_idle(100,100);
			// bc3.setRobotVelocity_idle(250,250);
          		/*bc4.setRobotVelocity_idle(250,250);
          		bc5.setRobotVelocity_idle(250,250);*/
			 ArUtil::sleep(300);
          		bc.getAngles(0);
          		bc1.getAngles(1);
         		 bc2.getAngles(2);
			//bc3.getAngles(3);
			omnicx = omnicx+ra[0][3]*0.3;
			omnicy = omnicy+ra[0][4]*0.3;
			/*bc.setRobotVelocity_idle(0,0);
         		 bc1.setRobotVelocity_idle(0,0);
	  		bc2.setRobotVelocity_idle(0,0);
			 bc3.setRobotVelocity_idle(0,0);*/
		//	omnicx = ra[0][1]+ra[0][5]*cos(ra[0][0]);
//omnicy = ra[0][2]+ra[0][5]*sin(ra[0][0]);
			printf("omnicx = %f, omnicy = %f\n",omnicx,omnicy);
			printf("x = %d",f);
		/*	bc3.getAngles(3);
			bc4.getAngles(4);
			bc5.getAngles(5);*/
	}
	return bc.disconnect();
}
