package wallFollower;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {
	
	
	public int PFactor =6;
	
	private final int bandCenter, bandwidth;
	private final int FILTER_OUT = 25;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int distance;
	private int filterControl;
	
	public static final int SINTERVAL=100;		// A 10Hz sampling rate
	public static final double PROPCONST=10.0;	// Proportionality constant
	//public static final int bandCenter=30; 		// Distance to wall * 1.4 (cm)
	public static final int MAXDIST = 200;
	
	public static final int FWDSPEED=200;		// Forward speed (deg/sec)
	
	public static final int MAXCORRECTION=200;	// Bound on correction to prevent stalling
	
	public static TextLCD t = LocalEV3.get().getTextLCD();					// n.b. how the screen is accessed

	public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
					   int bandCenter, int bandwidth) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		filterControl = 0;
	}
	
	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		
		int error =0;// = this.distance - bandCenter;
		int leftSpeed = FWDSPEED;
		int rightSpeed= 0;
		
		error = (this.distance - bandCenter);	
		
		// whether wheel should be set to reverse or forwards
		boolean reverse =false;

		// Controller Actions 
		if (Math.abs(error) <= bandwidth) {		// Case 1: Error in bounds, no correction 
			rightSpeed = FWDSPEED;				 
		}
		else{												//Case 2, we need to turn
			leftSpeed =FWDSPEED;							//Keep left wheel at constant speed and only modify right wheel
			rightSpeed = FWDSPEED * (1 + (error / PFactor));	//Right wheel speed is modified by a direct proportionality factor, which may result in negative speeds (which is expected and wanted)
			
			if(	rightSpeed > FWDSPEED+MAXCORRECTION){		//We cap the max speed of the right wheel in the forwards direction by the forward speed + maxCorrection
				rightSpeed =  FWDSPEED+MAXCORRECTION;
			}else if( rightSpeed < (-1*MAXCORRECTION)){		//We cap the max speed of the right wheel in reverse direction by the maxCorrection, meaning our reverse speed will never be as large as our forward speed 
				rightSpeed = -1*MAXCORRECTION;
			}
			
			if(rightSpeed<0){								//Flag right wheel to reverse and make speed positive.
				reverse = true;
				rightSpeed = (rightSpeed*-1);
			}
		}
				
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		
		leftMotor.forward();
		if(!reverse){
			rightMotor.forward();
		}
		else
		{
			rightMotor.backward();
		}
	}

	
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
