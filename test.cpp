#include "botConnector.h"

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
vector<int> BotConnector::getAngles()
{

	vector<int> angle;
	int count = robot.getNumSonar();
	const ArRobotParams *roboParams = robot.getRobotParams();
	printf("robot angle=%f, robot pose=(%f, %f)\n",robot.getTh(), robot.getX(), robot.getY());
	for(int i=0; i<count;i++)
	{
		int sonarDir = roboParams->getSonarTh(i)+round(robot.getTh());
		angle.push_back(sonarDir);
		//        printf("angle[%d]=%d\t",i,sonarDir);
	}
	printf("\n");
	return angle;
}

/**
 * moveRobotTo() rotates the robot by the mentioned theta and displaces the robot
 * by the given distance in the new heading.
 * @args:
 * 	double r - the distance in Millimeter by which robot is to be moved. Default=1000.
 * 	double th - the angle in degrees by which the robot is to be rotated. Default=45
 * @return: None
 */
void BotConnector::moveRobot(double r, double th)
{
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

	//    ArLog::log(ArLog::Normal, "Going to four goals in turn for %d seconds, then cancelling goal and exiting.", duration/1000);
	//    ArTime start;
	//    start.setToNow();
	//    while(Aria::getRunning())
	//    {
	//        if(robot.isHeadingDone())
	//        {
	//            robot.unlock();
	//            printf("turned in %ld\n",start.mSecSince());
	//            break;
	//        }
	//        gotoPoseAction->setGoal(pos);

	//        if(gotoPoseAction->haveAchievedGoal())
	//        if(start.mSecSince() >= duration)
	//        {
	//            gotoPoseAction->cancelGoal();
	//            printf("time out :(\n");
	//            break;
	//        }
	//        else if(gotoPoseAction->haveAchievedGoal())
	//        {
	//            gotoPoseAction->cancelGoal();
	//            printf("task in time %ld\n",start.mSecSince());
	//            break;
	//        }
	//    }
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

int main(int argc, char **argv)
{
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

//	bc.moveRobot(2000.0, 45.0);
//	vector<int> readings = bc.getReadings();
//	for( int i=0; i<readings.size(); i++ )
//	{
//		printf("%d\t", readings[i]);
//	}
//	printf("\n");
//	printf("Sleeping for 5 seconds\n");
//	ArUtil::sleep(5000);
	bc.setRobotVelocity(200, 200, 10000);

	return bc.disconnect();
}
