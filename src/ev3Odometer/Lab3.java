/**
 * 
 * Lab3 navigation and wall avoidance
 * 
 * @author Elie Harfouche and Guillaume Martin-Achard
 * @since  2016-10-10
 */
package ev3Odometer;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
public class Lab3 {
	
	
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	private static final EV3LargeRegulatedMotor usMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15.8;
	
	public static double[][] waypointsPartA = {{60.0,30.0},{30.0,30.0},{30.0,60.0},{60.0,0.0}};
	public static double[][] waypointsPartB = {{0.0,60.0},{60.0,0.0}};
	
	public static void main(String[] args) {
		
		//Initialize display on EV3
		final TextLCD t = LocalEV3.get().getTextLCD();

		//Initialize odometer and odometer display
		Odometer odometer = new Odometer(leftMotor, rightMotor,WHEEL_RADIUS,TRACK);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
		
		//Initialize ultrasonic sensor
		EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S1"));
			
		
		//Initialize both navigators and pass them their specific path
		AbNavigator regNavi = new Navigator(leftMotor,rightMotor,odometer,WHEEL_RADIUS,WHEEL_RADIUS,TRACK,waypointsPartA);
		AbNavigator avoidNavigator = new NavigatorAvoidance(leftMotor,rightMotor,odometer,WHEEL_RADIUS,
				WHEEL_RADIUS,TRACK,waypointsPartB,usSensor,usMotor);
	
		do {			
			
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left    | Right >   ", 0, 0);
			t.drawString("          |           ", 0, 1);
			t.drawString("Regular   | Avoidance ", 0, 2);
			t.drawString("Navigator | Navigator ", 0, 3);
						
			int choice  = Button.waitForAnyPress();
			
			
			//Regular navigation
			if(choice==Button.ID_LEFT)
			{
				odometer.start();
				odometryDisplay.start();
				regNavi.start();
				
			}
			//Avoidance navigation
			else if(choice==Button.ID_RIGHT)
			{
				odometer.start();
				odometryDisplay.start();
				avoidNavigator.start();
			}
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}