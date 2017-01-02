/**
 *
 * Auto using 4962 Hardware
 *
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;


/**
 *
 */
@Autonomous(name = "Test Robot 4962 - auto red US", group = "Team")
//@Disabled
public class TestRobot4962_auto_red_US extends LinearOpMode {

	// Get the robot hardware
	Hardware4962 robot  = new Hardware4962();

	// declare the optical distance sensor -- used to find lines on the floor

	//OpticalDistanceSensor odsSensor;

	// declare the color sensor -- used to find the color of the beacons

	byte[] colorCcache;

	I2cDevice colorC;
	I2cDeviceSynch colorCreader;


	// Modern Robotics Range Sensor

	byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

	I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
	public static final int RANGE1_REG_START = 0x04; //Register to start reading
	public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

	public I2cDevice RANGE1;
	public I2cDeviceSynch RANGE1Reader;


	// navX - values for initialization (from example code)
	//     this is an I2C sensor with a special driver from Kauai Labs.


	private final int NAVX_DIM_I2C_PORT = 0; // port on Core Device Interpace Module it's plugged in
	private AHRS navx_device;
	private navXPIDController yawPIDController;
	private ElapsedTime runtime = new ElapsedTime();

	private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

	private double TARGET_ANGLE_DEGREES = 0.0;	// this changes all the time, so just a starting #
	private final double TOLERANCE_DEGREES = 1.0;
	private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
	private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;

	// These are the PID tuning values. We just use the proportional part of their code
	private final double YAW_PID_P = 0.01; // this might change depending on if we are turning or
	// going straight
	private final double YAW_PID_I = 0.0;
	private final double YAW_PID_D = 0.0;

	private boolean calibration_complete = false;



	@Override
	public void runOpMode() throws InterruptedException {

		// use the Hardware4962 stuff that decribes the robot.
		robot.init(hardwareMap);

		// hardware and initialize the sensors
		//odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
		colorC = hardwareMap.i2cDevice.get("color sensor");
		colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x1e), false);
		colorCreader.engage();

		// Modern Robotics Range Sensor setup

		RANGE1 = hardwareMap.i2cDevice.get("range");
		RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
		RANGE1Reader.engage();

		// navX setup

		navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
				NAVX_DIM_I2C_PORT,
				AHRS.DeviceDataType.kProcessedData,
				NAVX_DEVICE_UPDATE_RATE_HZ);


        /* Create a PID Controller which uses the Yaw Angle as input. */
		yawPIDController = new navXPIDController( navx_device,
				navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
		yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
		yawPIDController.setContinuous(true);
		yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
		yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
		yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
		yawPIDController.enable(true);


		// color sensor initilization

		colorCreader.write8(3, 1);    //Set the mode of the color sensor to passive (0=active)
		// passive = turn off the LED

		// intake not running.

		double intakePower = 0.0;

		// wait for the start button to be pressed.
		waitForStart();

		// Make sure the navX is ready to be used.

		while ( !calibration_complete && opModeIsActive()) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
			calibration_complete = !navx_device.isCalibrating();
			if (!calibration_complete) {
				telemetry.addData("navX-Micro", "Startup Calibration in Progress");
				telemetry.update();
			}
		}
		// start at a heading of zero.

		navx_device.zeroYaw();
		int DEVICE_TIMEOUT_MS = 500;
		navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

		DecimalFormat df = new DecimalFormat("#.##");


		/*************************************************************************************/
		/*************************************************************************************/
		/*
		/*  start of code that does stuff
		 */



		// Drive parallel to the ramp and then turn parallel to the wall at the first beacon.

		DriveOnHeading(yawPIDResult,0,3);
		TurnToHeading(yawPIDResult, -45., 2.0);
		DriveOnHeading(yawPIDResult,-45,43);
		TurnToHeading(yawPIDResult, 0., 2.0);

		// Drive forward until we see red

		robot.Drive(0.15,0.15);
		readColor();
		while(!colorIsRed() && opModeIsActive()) {
			telemetry.addData("red","red = " + colorIsRed());
			telemetry.addData("blue","blue = " + colorIsBlue());
			telemetry.update();
			readColor();
		}
		sleep(500);
		robot.StopDriving();

		// extend and retract button pusher

		robot.button.setPosition(0.7);
		sleep(4000);
		robot.button.setPosition(0.0);
		sleep(1000);

		// drive toward second beacon parallel to the wall

		//DriveOnHeading(yawPIDResult,-1,30);
		DriveAlongWall(11,30);

		// slow down and look for the beacon

		robot.Drive(0.15,0.15);
		readColor();
		while(!colorIsRed() && opModeIsActive()) {
			telemetry.addData("red","red = " + colorIsRed());
			telemetry.addData("blue","blue = " + colorIsBlue());
			telemetry.update();
			readColor();
		}
		//sleep(300);



		telemetry.addData("b1","driving to button");
		telemetry.update();
		robot.StopDriving();
		telemetry.addData("b2","pushing button");
		telemetry.update();

		// push the correct button

		robot.button.setPosition(0.7);
		sleep(4000);
		telemetry.addData("b3","done with button");
		telemetry.update();
		robot.button.setPosition(0.0);
		sleep(1000);

		// turn with back of robot towards the center vortex

		TurnToHeading(yawPIDResult, -45., 2.0);

		// drive in reverse to hit the ball

		DriveOnHeadingReverse(yawPIDResult,-45,53);
		robot.StopDriving();


	}


	// read the color from the I2C Modern Robotics Color Sensor

	public void readColor(){
		colorCcache = colorCreader.read(0x04, 1);
	}


	// return true if the color is a shade of red

	boolean colorIsRed() {
		if ((colorCcache[0] & 0xFF) == 10 ||
				(colorCcache[0] & 0xFF) == 11) {
			return (true);
		}
		else return (false);
	}

	// return true if the color is a shade of blue

	boolean colorIsBlue() {
		if ((colorCcache[0] & 0xFF) == 2 ||
				(colorCcache[0] & 0xFF) == 3) {
			return (true);
		} else return (false);
	}


	public double limit(double a) {
		return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
	}

	// Turn toward an exact heading using the naxV PID controller.

	public void TurnToHeading(navXPIDController.PIDResult yawPIDResult, double heading, double maxTimeSeconds) {
		try {
			long starttime= System.currentTimeMillis();

			TARGET_ANGLE_DEGREES = heading;
			int DEVICE_TIMEOUT_MS = 500;
			yawPIDController.setPID(0.02, YAW_PID_I, YAW_PID_D);
			yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
			DecimalFormat df = new DecimalFormat("#.##");

			while (
					!Thread.currentThread().isInterrupted()&& (System.currentTimeMillis()-starttime) < (long) (maxTimeSeconds*1000) ) {
				if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
					if (yawPIDResult.isOnTarget()) {
						float zero = 0;
						robot.FloatMotors();
						telemetry.addData("PIDOutput", df.format(0.00));
						telemetry.update();
					} else {
						double output = yawPIDResult.getOutput();
						if (output > 0 && output < 0.25) { output = 0.25; }
						if (output < 0 && output > -0.25) { output = -0.25; }
						robot.Drive(output, -output);
						telemetry.addData("PIDOutput", df.format(output) + ", " +
								df.format(-output));
					}
				} else {
			    /* A timeout occurred */
					Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
					telemetry.addData("timeout","timeout occured");
				}
				telemetry.addData("Yaw aw", df.format(navx_device.getYaw()));
				telemetry.update();
			}
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}

	}

	// Drive on an exact heading using the navX PID controller and encoders

	public void DriveOnHeading(navXPIDController.PIDResult yawPIDResult, double heading, double distanceInches) {

		// calculate encoder counts for distance
		float wheelDiameter = 4; // inches
		float wheelCirc = wheelDiameter* (float) 3.14159;
		float encoderTicksPerRotation = 1120;
		float ticksPerInch =encoderTicksPerRotation/wheelCirc;
		int ticksToTravel=(int) (distanceInches*ticksPerInch);

		// check motor position
		int startEncCount=robot.leftfrontMotor.getCurrentPosition();
		int DEVICE_TIMEOUT_MS = 500;
        /* Drive straight forward at 1/2 of full drive speed */
		double drive_speed = 0.5;
		yawPIDController.setPID(0.05, YAW_PID_I, YAW_PID_D);
		yawPIDController.setSetpoint(heading);
		DecimalFormat df = new DecimalFormat("#.##");
		try {
			while ((robot.leftfrontMotor.getCurrentPosition()-startEncCount) < ticksToTravel &&
					!Thread.currentThread().isInterrupted()) {
				if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
					if (yawPIDResult.isOnTarget()) {
						robot.Drive(drive_speed, drive_speed);
						telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
								df.format(drive_speed));
					} else {
						double output = yawPIDResult.getOutput();
						if (output < -0.5) {
							output = -0.5;
						}
						if (output > 0.5) {
							output = 0.5;
						}
						robot.Drive(drive_speed + output, drive_speed - output);
						telemetry.addData("PIDOutput", df.format(limit(-drive_speed - output)) + ", " +
								df.format(limit(-drive_speed + output)));
					}
					telemetry.addData("Yaw", df.format(navx_device.getYaw()));
					telemetry.addData("Output", df.format(yawPIDResult.getOutput()));
					telemetry.addData("Enc:", robot.leftfrontMotor.getCurrentPosition());
					telemetry.addData("EncStart:", startEncCount);
					telemetry.update();
				} else {
			        /* A timeout occurred */
					Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
				}
			}
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
	}

	// drive on a heading in reverse using the naxV PID controller and encoders

	public void DriveOnHeadingReverse(navXPIDController.PIDResult yawPIDResult, float heading, float distanceInches) {

		// calculate encoder counts for distance
		float wheelDiameter = 4; // inches
		float wheelCirc = wheelDiameter* (float) 3.14159;
		float encoderTicksPerRotation = 1120;
		float ticksPerInch =encoderTicksPerRotation/wheelCirc;
		int ticksToTravel=(int) (distanceInches*ticksPerInch);

		// check motor position
		int startEncCount=robot.leftfrontMotor.getCurrentPosition();
		int DEVICE_TIMEOUT_MS = 500;
        /* Drive straight forward at 1/2 of full drive speed */
		double drive_speed = 0.5;
		yawPIDController.setPID(0.05, YAW_PID_I, YAW_PID_D);
		yawPIDController.setSetpoint(heading);
		DecimalFormat df = new DecimalFormat("#.##");
		try {
			while ((robot.leftfrontMotor.getCurrentPosition()-startEncCount) > -ticksToTravel &&
					!Thread.currentThread().isInterrupted()) {
				if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
					if (yawPIDResult.isOnTarget()) {
						robot.Drive(-drive_speed, -drive_speed);
						telemetry.addData("PIDOutput", df.format(drive_speed) + ", " +
								df.format(drive_speed));
					} else {
						double output = yawPIDResult.getOutput();
						if (output < -0.5) {
							output = -0.5;
						}
						if (output > 0.5) {
							output = 0.5;
						}
						robot.Drive(-drive_speed + output, -drive_speed - output);

						telemetry.addData("PIDOutput", df.format(limit(-drive_speed - output)) + ", " +
								df.format(limit(-drive_speed + output)));
					}
					telemetry.addData("Yaw", df.format(navx_device.getYaw()));
					telemetry.addData("Output", df.format(yawPIDResult.getOutput()));
					telemetry.addData("RevEnc:", robot.leftfrontMotor.getCurrentPosition());
					telemetry.addData("EncStart:", startEncCount);
				} else {
			        /* A timeout occurred */
					Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
				}
			}
		}
		catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
	}

	public void DriveAlongWall (double distanceFromWallCm, double distanceToTarget) {

		// calculate encoder counts for distance
		float wheelDiameter = 4; // inches
		float wheelCirc = wheelDiameter* (float) 3.14159;
		float encoderTicksPerRotation = 1120;
		float ticksPerInch =encoderTicksPerRotation/wheelCirc;
		int ticksToTravel=(int) (distanceToTarget*ticksPerInch);

		// check motor position
		int startEncCount=robot.leftfrontMotor.getCurrentPosition();

		// set base drive speed
		double drive_speed = 0.3;
		double gain = 6.0;

		while ((robot.leftfrontMotor.getCurrentPosition()-startEncCount) < ticksToTravel &&
				!Thread.currentThread().isInterrupted()) {
			// get range from the target, which is the wall
			range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
			double range = (double) (range1Cache[0] & 0xFF);
			double correction = 0.01*(range-distanceFromWallCm)*gain;
			double right = drive_speed+correction;
			double left = drive_speed-correction;
			if (left > 1.0) { left = 1.0; }
			if (right > 1.0) { right = 1.0; }
			if (left < -1.0) { left = -1.0; }
			if (right < -1.0) { right = -1.0; }
			robot.Drive(left,right);
			telemetry.addData("Range:", range);
			telemetry.addData("left", left);
			telemetry.addData("right", right);
			telemetry.addData("correction",correction);

			telemetry.update();
		}

	}
}