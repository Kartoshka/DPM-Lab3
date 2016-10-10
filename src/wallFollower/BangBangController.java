package wallFollower;
import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int fwdSpeed = 200;
	
	/**
	 * Speeds for the left and right motor based on which direction we are turning
	 * Turn speed for the directions is different, because it allows us to handle going around walls
	 */
	private final int leftMotorTL = (int)(200*0.8f);
	private final int rightMotorTL = (int)(200*1.5f);
	
	private final int leftMotorTR = (int)(200*1.5f);
	private final int rightMotorTR= (int)(200*0.6f);
	
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	/**
	 * Bang Bang controller adjusts speed by a set amount regardless of the error
	 * @param leftMotor
	 * @param rightMotor
	 * @param bandCenter distance to keep from wall
	 * @param bandwidth	 accepted error
	 */
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth) {
		//Default Constructor
		this.bandCenter = bandCenter; //Distance from wall
		this.bandwidth = bandwidth;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	

	/*
	 * Speeds are based on real tests
	 * @see wallFollower.UltrasonicController#processUSData(int)
	 */
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		
		int error = this.distance - bandCenter; //Calculate error
		
		float lmSpeed  =fwdSpeed;
		float rmSpeed = fwdSpeed;
		
		if (Math.abs(error)<=bandwidth){		//If error is within acceptable bounds, go forwards
			lmSpeed = rmSpeed =fwdSpeed;
		}
		else if(this.distance<bandCenter*0.5f){  //If we are very very close to the wall, make a very sharp turn (reversing one motor)
			lmSpeed = leftMotorTR*1.5f;
			rmSpeed = rightMotorTR*(-0.1f);
		}
		else if (this.distance<bandCenter){    //If we are too close to the wall, turn left
			lmSpeed = leftMotorTR;
			rmSpeed = rightMotorTR;

		}
		else if (this.distance>bandCenter){	  //If we are too far from the wall, turn right 
			lmSpeed = leftMotorTL;
			rmSpeed = rightMotorTL;
		}
		
		leftMotor.setSpeed(lmSpeed);
		rightMotor.setSpeed(Math.abs(rmSpeed));
	
		leftMotor.forward();
		
		if(rmSpeed>0){						//If right motor has negative speed, we have to reverse the motor
			rightMotor.forward();
		}
		else{
			rightMotor.backward();
		}
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
