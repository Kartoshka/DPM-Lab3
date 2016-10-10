/**
 * Abstract class for navigator that extends Thread.
 * 
 * Initialize it with proper variables and a list of waypoints to travel to then begin it as a separate 
 * thread in order for the robot to travel to set destinations in order.
 * Initializes motors, navigation and rotation flags, and waypoints through common constructor.
 * Provides common methods to rotate and stop robot wheels.
 * Concrete children must implement TravelTo() method which is called by the run method.
 * 
 * @author Elie Harfouche and Guillaume Martin-Achard
 * @since  2016-10-10
 */
package ev3Odometer;

import java.util.LinkedList;
import java.util.Queue;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public abstract class AbNavigator extends Thread{

	//Accepted error from final destination. I.E. if we are within 1.0cm of final destination, stop
	protected static final float ACCEPTED_THRESHOLD = 1.0f;
	//Aceptable deviation of rotation angle during navigation
	protected static final double ANGLE_ERROR_THRESHHOLD = Math.toRadians(5);
	
	
	//Minimum number of ms to wait between each loop within travelTo
	protected static final long TIME_THREAD=16;
	
	//Forward and rotational speeds of robot
	protected static final int FORWARD_SPEED = 250; 
	protected static final int ROTATE_SPEED = 150; 
	
	//Flags to indicate whether robot is rotating or navigating
	protected boolean isNavigating = false;
	protected boolean isRotating = false;
	
	//Motors
	protected EV3LargeRegulatedMotor lMotor;
	protected EV3LargeRegulatedMotor rMotor;
	
	//Radius of left and right wheels
	protected double lWR;
	protected double rWR;
	
	//Distance between wheels
	protected double trackDist;
	
	//Odometer to retrieve real time positional information
	protected Odometer odometer;
	
	//List of coordinates to travel to in order
	protected Queue<double[]> waypoints = new LinkedList<double[]>();

	//Heading of the robot updated real time as it travels
	protected static double currentHeading =0.0f;
	//Current position of the robot updated as it travels. appears as {x,y} 
	protected static double[] currentPos = {0,0f,0,0f};
	
	
	public AbNavigator(EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor,Odometer odometer,double leftRadius, double rightRadius, double width, double[][] destinations )
	{
		//retrieve motor information
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.odometer = odometer;
		//Retrieve wheel and track information
		lWR = leftRadius;
		rWR = rightRadius;
		trackDist = width;
		//Update current coordinates in space
		currentHeading = odometer.getTheta();
		currentPos[0] = odometer.getX();
		currentPos[1] = odometer.getY();
		//Queue up waypoints to travel to 
		for(double[] coords:destinations){
			waypoints.add(coords);
		}
		
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
	
	/**
	 * Order robot to travel to given coordinates in space 
	 * @param coordinates Coordinates in space (X,Y) cm to travel to
	 */
	protected abstract void TravelTo(double[] coordinates);
	
	/**
	 * Verify whether robot is currently in motion.
	 * @return whether robot is in motion
	 */
	public boolean isNavigating(){
		//Returns whether robot is rotating or navigating. Flags must be set by sublcasses.
		return isRotating || isNavigating;
	}
	
	/**
	 * Turns robot by a given angle clockwise.
	 * @param theta Angle to turn by in degrees
	 */
	protected void TurnToAngle(double theta){
		//Flag that we are rotating
		isRotating=true;
		
		//Convert angle to radians
		theta = Math.toRadians(theta);
		
		//Set motor speeds to rotation speeds	
		lMotor.setSpeed(ROTATE_SPEED);
		rMotor.setSpeed(ROTATE_SPEED);
		
        //rotate wheels to turn, using a method to find how much it has to rotate
		lMotor.rotate(convertAngle(lWR, trackDist, theta), true);
		rMotor.rotate(-convertAngle(rWR, trackDist, theta), false);
		
		//Update current heading
		currentHeading = odometer.getTheta();
		
		//Flag that rotation has completed
		isRotating=false;
		
	}
	
	/**
	 * Convert distance to travel to a number of degrees for the wheel to rotate
	 * @param radius Radius of wheel
	 * @param distance Forward distance to travel
	 * @return degrees of rotation wheel should rotate
	 */
	protected static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Convert angle to turn by robot into angle to turn by wheels
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	protected static int convertAngle(double radius, double width, double angle) {
		return (int) ((width*(angle))/(2*radius));
	}
	
	/**
	 * Return smallest angle to rotate between two angles
	 * @param initialAngle starting angle
	 * @param finalAngle final angle
	 * @return smallest angle (CW) to rotate by (degrees) to reach finalAngle from initialAngle
	 */
	protected double getSmallestRotation(double initialAngle, double finalAngle){
		
		double diff = finalAngle - initialAngle;
		
		if(Math.abs(diff)<Math.PI)
		{
			return diff;
		}
		else if(diff<-Math.PI)
		{
			return diff+Math.PI*2;
		}
		else
		{
			return diff - Math.PI*2;
		}
	}
	
	/**
	 * Half motors without calling stop. Does not mess up the odometer or cause motors to reverse.
	 * Also runs faster than motor.stop();
	 */
	protected void haltMotors(){
		//Set speed to 0
		lMotor.setSpeed(0);
		rMotor.setSpeed(0);
		//Update speed
		lMotor.forward();
		rMotor.forward();
	}
	
	
}
