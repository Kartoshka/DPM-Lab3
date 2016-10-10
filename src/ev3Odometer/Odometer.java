/*
 * Odometer.java
 */

package ev3Odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * The Odometer class calculates the displacement of the robot relative to the center of two wheels based on their tachometer values.
 * It will compute the X,Y (cm) and rotation (degrees) of the robot since initialization
 * Odometer must be started as a separate Thread and implements thread safe variables during its processes
 * @author elieharfouche and guillaumemartin-achard
 *
 */
public class Odometer extends Thread {
	// robot positions
	private double x, y, theta;
	//Last computed tachometer counts
	private int leftMotorTachoCount, rightMotorTachoCount;
	//Motors polled
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	//Radius of the wheels
	private double wheelRadius;
	//Separation between wheels
	private double wheelsSeparation;
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor, double WheelRadius,double distanceBetweenWheels) {
		//Initialize all variables
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
		this.wheelRadius=WheelRadius;
		this.wheelsSeparation = distanceBetweenWheels;
	}

	//
	public void run() {
		long updateStart, updateEnd;

		
		while (true) {
			updateStart = System.currentTimeMillis();			

			//Lock in order to synchronize thraeds
			synchronized (lock) {
				
				//Get new tachometer values from motors
				int newLMTacho = leftMotor.getTachoCount();
				int newRMTacho = (int)(rightMotor.getTachoCount());
				
				//Calculate displacement of wheels since last poll based on current tachometer values - last tachometer values.
				//Values are passed to a function which returns a distance in cm
				double lWDisplacement = this.getDistanceByTachoCount(newLMTacho-this.getLeftMotorTachoCount());
				double rWDisplacement = this.getDistanceByTachoCount(newRMTacho-this.getRightMotorTachoCount());
				
				//Calculate the angle of the robot 
				//The angle is based on sin, as we are creating a triangle out of the wheels' old positions and new positions and their track distances.
				//However, as this calculation is done often, we can use small angle approximation of sin(theta) = theta in order to avoid calculations
				double theta = (lWDisplacement-rWDisplacement)/this.wheelsSeparation;
				//If theta is negative, we transform it to its positive value (360-theta)
				if(theta<0){
					theta = Math.PI*2 +theta;
				}
				
				//Add angle turn to global robot angle
				this.theta+=theta;
				//Calculate the displacement of the overall robot centered between the two wheels
				double displacementRobot = (lWDisplacement+rWDisplacement)/2;
				
				//Calculate the instantaenous displacement in X and Y of the robot based on the overall displacement and the header angle
				double dx = displacementRobot*Math.sin(this.theta);
				double dy = displacementRobot*Math.cos(this.theta);
				
				//Add instantaneous displacement to overall position
				x+=dx;
				y+=dy;
				
				//Update tacometer values
				this.setLeftMotorTachoCount(newLMTacho);
				this.setRightMotorTachoCount(newRMTacho);
				
				//Keep theta between [0,2*pi) aka [0,360]
				this.theta %=2*Math.PI;
				
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors for private methods
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators for private variables
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;	
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;	
		}
	}
	
	/**
	 * Calculate the distance travelled of a single wheel based on given tachometer value.
	 * uses circumference of the wheel multiplied by a ratio of the arc it did
	 * @param tachoCount
	 * @return distance travelled 
	 */
	public double getDistanceByTachoCount(double tachoCount){
		return ((this.wheelRadius*Math.PI*tachoCount)/180.0);
	}
}
