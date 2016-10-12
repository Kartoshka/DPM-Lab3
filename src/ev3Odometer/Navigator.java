/**
 * 
 * Navigator which travels to all points passed in constructor
 * 
 * @author Elie Harfouche and Guillaume Martin-Achard
 * @since  2016-10-10
 */

package ev3Odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends AbNavigator{
	
	private static final float ACCEPTED_THRESHOLD =1.0f;
	private static final float ANGLE_ERROR_THRESHHOLD = (float)Math.PI*5/180f;

	public Navigator(EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor,Odometer odometer,double leftRadius, double rightRadius, double width, double[][] destinations )
	{
		super(lMotor,rMotor,odometer,leftRadius,rightRadius,width,destinations);
	}
	
	@Override
	 public void run(){
		
		//loop through all given waypoints and travel to them in order.
		while(waypoints.size()>0){
			TravelTo(waypoints.poll());
		}
		
		//Once we have finished travelling, stop motors.
		lMotor.stop();
		rMotor.stop();
	}
	
	@Override
	protected void TravelTo(double[] coordinates){
		isNavigating =true;
		
		//Set Motor speeds forward
			
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
			
			//Bring the expected angle within a range of [0,2PI) to match the readings of our odometer
			if(vectorTheta<0){
				vectorTheta+=Math.PI*2;
			}
			
			double rotationError = Math.abs(getSmallestRotation(currentHeading,vectorTheta));
			
			if(rotationError>ANGLE_ERROR_THRESHHOLD){
				TurnToAngle(getSmallestRotation(currentHeading,vectorTheta));
			}
			
			//If we have passed all the checks, then we are facing no obstacles and in a direct line to the wall
			//WE set motors to forward speed and advance
			lMotor.setSpeed(FORWARD_SPEED);
			rMotor.setSpeed(FORWARD_SPEED);
			
			lMotor.forward();
			rMotor.forward();
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
			
			//get latest updated position from odometer
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


