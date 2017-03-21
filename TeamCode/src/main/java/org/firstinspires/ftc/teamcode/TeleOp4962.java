/**
 *
 * TeleOp using 4962 hardware
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 *
 */
@TeleOp(name = "TeleOp 4962a", group = "TeleOp")
//@Disabled
public class TeleOp4962 extends LinearOpMode {

	Hardware4962 robot= new Hardware4962();		// this is our hardware class

	@Override
	public void runOpMode() throws InterruptedException {

		// use the Hardware4962 stuff that decribes the robot.
		robot.init(hardwareMap);

		double intakePower = 0.0;

		// wait for the start button to be pressed.
		waitForStart();

		int encoderCountFront = 0;
		int encoderCountBack = 0;

		while (opModeIsActive()) {
		 /*
		 * Gamepad 1 controls the motors via the left and right stick
		 */

			float left = gamepad1.left_stick_y;
			float right = gamepad1.right_stick_y;
			boolean intakeIn = gamepad1.left_bumper;
			boolean intakeOut = gamepad1.right_bumper;
			float elevatorPower = gamepad2.right_stick_y;
			boolean kickme = gamepad2.y;
			boolean shootme = gamepad2.right_bumper;
			double shootoff = gamepad2.right_trigger;
			boolean launchme = gamepad2.left_bumper;
			double nudgeme = gamepad2.left_trigger;
			double turbo = gamepad1.left_trigger;
			boolean shootlong = gamepad2.x;
			boolean sidebutton = gamepad1.x;


			// clip the right/left values so that the values never exceed +/- 1
			right = Range.clip(right, -1, 1);
			left = Range.clip(left, -1, 1);
			if(turbo > 0.5){

				right = right * (float)0.7;
				left =left * (float)0.7;
			}

			// scale the joystick value to make it easier to control
			// the robot more precisely at slower speeds.
			right = -(float) scaleInput(right);
			left = -(float) scaleInput(left);
			if (intakeIn){
				intakePower = 0.5;
			}
			if (intakeOut){
				intakePower = -1.0;
			}
			if (!intakeIn && !intakeOut){
				intakePower =0;
			}
			if (kickme) {
				robot.kicker.setPosition(0);
			} else {
				robot.kicker.setPosition(0.59);
			}
			//robot.shooter.setPower(0.9);
			if (shootme) {
				robot.ShooterSpeed(0.8,0.4); // backspin
				//robot.ShooterSpeed(0.55);
			}
			if (shootlong) {
				robot.ShooterSpeed(.6,.9);
			}
			if (shootoff > 0.5){
				robot.ShooterSpeed(0);
			}
			if (launchme) {
				robot.launch.setPosition(0.05);
			} else {
				robot.launch.setPosition(0.31);
			}
			if (nudgeme > 0.5) {
				robot.launch.setPosition(0.16);
			}
			if (sidebutton) {
				robot.button.setPosition(0.4);
			}
			else {
				robot.button.setPosition(0.0);
			}
			// write the values to the motors
			robot.Drive(left,right);

			robot.intakeMotor.setPower(intakePower);
			robot.elevatorMotor.setPower(elevatorPower);

			// get RPM

			int encoderFrontDiff = robot.frontshooterMotor.getCurrentPosition() - encoderCountFront;
			int encoderBackDiff = robot.backshooterMotor.getCurrentPosition() - encoderCountBack;
			encoderCountFront = robot.frontshooterMotor.getCurrentPosition();
			encoderCountBack = robot.backshooterMotor.getCurrentPosition();
			telemetry.addData("Text", "*** Robot Data***");
			telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
			telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
			telemetry.addData("elevator", "e pwr = " + String.format("%.2f", elevatorPower));
			//telemetry.addData("shooter RPM","enc cnt F B = " +  encoderFrontDiff + "/" + encoderBackDiff);
			telemetry.addData("shooter","enc cnt F B = " +  robot.frontshooterMotor.getCurrentPosition() + "/" + robot.backshooterMotor.getCurrentPosition());
			telemetry.update();
			idle();
		}
	}



	/*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) (dVal * 16.0);

		// index should be positive.
		if (index < 0) {
			index = -index;
		}

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}

}
