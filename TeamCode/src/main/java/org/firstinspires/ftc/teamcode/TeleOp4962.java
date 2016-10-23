/**
 *
 * Simple arcade drive for two motors and nothing else
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
@TeleOp(name = "TeleOp Tank 4962", group = "TeleOp")
//@Disabled
public class TeleOp4962 extends LinearOpMode {

    Hardware4962 robot;		// this is our hardware class

	@Override
	public void runOpMode() throws InterruptedException {

		// use the Hardware4962 stuff that decribes the robot.
		robot.init(hardwareMap);

		double intakePower = 0.0;

		// wait for the start button to be pressed.
		waitForStart();

		while (opModeIsActive()) {
		 /*
		 * Gamepad 1 controls the motors via the left stick
		 */

			// throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
			// 1 is full down
			// direction: left_stick_x ranges from -1 to 1, where -1 is full left
			// and 1 is full right
			//float throttle = -gamepad1.left_stick_y;
			//float direction = gamepad1.left_stick_x;
			//float right = throttle + direction;
			//float left = throttle - direction;
			float left = -gamepad1.left_stick_y;
			float right = -gamepad1.right_stick_y;
			boolean intakeIn = gamepad1.left_bumper;
			boolean intakeOut = gamepad1.right_bumper;
			float elevatorPower = gamepad2.right_stick_y;

			// clip the right/left values so that the values never exceed +/- 1
			right = Range.clip(right, -1, 1);
			left = Range.clip(left, -1, 1);

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
			// write the values to the motors
			robot.Drive(left,right);

			robot.intakeMotor.setPower(intakePower);
			robot.elevatorMotor.setPower(elevatorPower);

			telemetry.addData("Text", "*** Robot Data***");
			telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
			telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
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
