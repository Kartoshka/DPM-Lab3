package ev3Odometer;


import wallFollower.UltrasonicController;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class NavigatorAvoidance extends AbNavigator {

	//Distance at which to detect obstacle
	private static final float WALL_DETECTION_DISTANCE =12;
	//Angle at which to rotate ultrasonic sensor once in wall avoidance mode
	private static final int US_SENSOR_BEHIND_ANGLE = 75;


	//Angle of ultra sonic sensor when in navigation mode
	private static final int FRONT_ANGLE_US_MOTOR =30;
	
	//Interval in which robot will accept that it is facing right direction when in obstacle avoidance mode
	private static final double AVOIDANCE_ACCEPTED_ANGLE_ERROR = Math.toRadians(15);
	//Minimum amount of distance for robot to detect it has turned corner during wall avoidance
	private static final double AVOIDANCE_ACCEPTED_WALL_DISTANCE = 50;
	
	//Wall follower code
	private static UltrasonicController wallFollower;
	
	//Angle at which robot will turn before initiating wall avoidance mode
	private final static double ROBOT_TURN_BY = Math.toRadians(90);
	
	
	private EV3LargeRegulatedMotor usMotor;
	private SampleProvider usSampler; 
	private float[] usData;
	
	//Flag whether we are in wall avoidance mode or not
	private boolean avoidWall =false;
	
	public NavigatorAvoidance(EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor,Odometer odometer,double leftRadius, 
			double rightRadius, double width, double[][] destinations,EV3UltrasonicSensor usSensor,EV3LargeRegulatedMotor usMotor)
	{
		super(lMotor,rMotor,odometer,leftRadius,rightRadius,width,destinations);
		
		usSampler = usSensor.getMode("Distance");
		usData = new float[usSensor.sampleSize()];
		this.usMotor = usMotor;
		
		this.usMotor.rotateTo(FRONT_ANGLE_US_MOTOR);
		
	}
	
	@Override
	protected void TravelTo(double[] coordinates){
		
		//Flag that we are currently in transit
		isNavigating =true;
		
		//This variable will hold the angle which the robot was travelling at before encountering a wall. it is used to detect when the robot is able to resume its travel
		double thetaBeforeWall=0;
		
		//We remain in the while loop while our current distance relative to our expected destination is larger than the accepted threshhold		
		while(Math.sqrt((Math.pow(currentPos[0]-coordinates[0], 2)+Math.pow(currentPos[1]-coordinates[1], 2))) >ACCEPTED_THRESHOLD){
			//Retrieve time at the beginning of the loop in order to keep update at a maximum refresh rate
			long startTime = System.currentTimeMillis();

			//Calculate the vector going from our current position to the destination
			double[] vectorDisplacement = new double[2];
			vectorDisplacement[0] = coordinates[0] -currentPos[0];
			vectorDisplacement[1] = coordinates[1] -currentPos[1];

			//We calculate the angle (0 deg being north, 90 being East, 180 South, and 270 West) the robot should be at in order to go in a straight line to the destination
			//Thanks to java's built in arctan function, angle is already capped between [-pi,pi)
			double vectorTheta = Math.atan2(vectorDisplacement[0], vectorDisplacement[1]);
			
			//WALL AVOIDANCE MODE
			if(avoidWall){
				
				//fetch our current distance from the wall. 
				//Note that, when avoidWall was flagged as true, the sensor was manually rotated
				usSampler.fetchSample(usData, 0);
				//Retrieve the current angle of the robot
				currentHeading = odometer.getTheta();
				
				//Calculate the error between our current angle and the angle we were at before wall avoidance turned on
				double errorAngle = Math.abs(getSmallestRotation(currentHeading,thetaBeforeWall));//Math.abs(currentHeading-thetaBeforeWall);
				
				
				//If we detect that the distance reported by the ultra sonic sensor is greater than the set threshhold (that is, we cleared a corner) 
				//and our current angle is within an acceptable range of our angle before wall avoidance (that is, we are facing the destination)
				// revert back to the navigation mode
				if(((int)(usData[0]*100.0)>=AVOIDANCE_ACCEPTED_WALL_DISTANCE && errorAngle<=(AVOIDANCE_ACCEPTED_ANGLE_ERROR)))
				{
					lMotor.setSpeed(0);
					rMotor.setSpeed(0);
					lMotor.forward();
					rMotor.forward();
					
					avoidWall=false;
					usMotor.rotateTo(30);
				}
				//Otherwise, notify the wall-follower and pass execution to that thread. 
				else{
					wallFollower.processUSData((int)(usData[0]*100.0));
				}
				
			}
			//NAVIGATION MODE
			else{
				
				//We have already calculated the necessary variables to determine our path before the if statement began.
				
				if(vectorTheta<0){
					vectorTheta+=Math.PI*2;
				}
				
				//Calculate the current error of our angle compared to the expected angle
				double smallestAngleToExpectd =getSmallestRotation(currentHeading,vectorTheta);
				double rotationError = Math.abs(smallestAngleToExpectd);//Math.abs(vectorTheta-currentHeading);
				
				//If we are not within our threshhold of error, rotate the robot to the correct angle
				if(rotationError>ANGLE_ERROR_THRESHHOLD){
					TurnToAngle(smallestAngleToExpectd);
				}
				
				
				//After we have rotated, we verify the ultra sonic sensor readings to see whether there is an obstacle in front of us
				usSampler.fetchSample(usData, 0);
				
				//If we detect a wall closer than an expected threshhold, we activate wall avoidance mode and skip the rest of this loop
				if((int)(usData[0]*100.0)<WALL_DETECTION_DISTANCE){
					
					//Set the angle that we currently are at, since this is facing our trajectory.
					thetaBeforeWall = currentHeading;
					
					lMotor.setSpeed(0);
					rMotor.setSpeed(0);
					lMotor.forward();
					rMotor.forward();
					
					//Rotate ultraSonic sensor to face to our side so that we can detect the wall
					usMotor.rotateTo(-US_SENSOR_BEHIND_ANGLE);
					
					//Turn robot to face away from wall
					TurnToAngle(ROBOT_TURN_BY);
					
					//Flag wall avoidance mode to begin
					avoidWall = true;
					
					//skip rest of navigation mode
					continue;
					
				}
				
				//If we have passed all the checks, then we are facing no obstacles and in a direct line to the wall
				//WE set motors to forward speed and advance
				lMotor.setSpeed(FORWARD_SPEED);
				rMotor.setSpeed(FORWARD_SPEED);
				
				lMotor.forward();
				rMotor.forward();
				
			}
			
			long endTime = System.currentTimeMillis();

			//Sleep thread if calculations were done faster than at a rate of faster than 16ms
			if((endTime-startTime)<TIME_THREAD){
				
				try {
					Thread.sleep(TIME_THREAD-(endTime-startTime));
				} 
				catch (InterruptedException e)
				{
						
				}
			}
			
			//get latest updated position from odometer after every loop iteration
			currentPos[0] = odometer.getX();
			currentPos[1] = odometer.getY();
			currentHeading = odometer.getTheta();
		}
		
		
		//Once we have arrived at given destination, we stop
		haltMotors();
		
		//Flag to say we are not currently navigating.
		isNavigating=false;
		
	}
}
