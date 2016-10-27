/**
 *
 * Simple arcade drive for two motors and nothing else
 *
 */

package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.text.DecimalFormat;


/**
 *
 */
@TeleOp(name = "Test Robot 4962 - auto", group = "Concept")
@Disabled
public class TestRobot4962_simpleauto extends LinearOpMode {

	Hardware4962 robot  = new Hardware4962();

	OpticalDistanceSensor odsSensor;

	byte[] colorCcache;

	I2cDevice colorC;
	I2cDeviceSynch colorCreader;


	// navX initialization

	/* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
	private final int NAVX_DIM_I2C_PORT = 0;
	private AHRS navx_device;
	private navXPIDController yawPIDController;
	private ElapsedTime runtime = new ElapsedTime();

	private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

	private double TARGET_ANGLE_DEGREES = 90.0;
	private final double TOLERANCE_DEGREES = 2.0;
	private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
	private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
	private final double YAW_PID_P = 0.005;
	private final double YAW_PID_I = 0.0;
	private final double YAW_PID_D = 0.0;

	private boolean calibration_complete = false;



	@Override
	public void runOpMode() throws InterruptedException {

		robot.init(hardwareMap);
		odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
		colorC = hardwareMap.i2cDevice.get("color sensor");
		colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x1e), false);
		colorCreader.engage();

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



		// color sensor initilization

		colorCreader.write8(3, 1);    //Set the mode of the color sensor to passive



		//robot.initSensors();
		double intakePower = 0.0;

		// wait for the start button to be pressed.
		waitForStart();

		while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
			calibration_complete = !navx_device.isCalibrating();
			if (!calibration_complete) {
				telemetry.addData("navX-Micro", "Startup Calibration in Progress");
			}
		}
		navx_device.zeroYaw();

		int DEVICE_TIMEOUT_MS = 500;
		navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

		DecimalFormat df = new DecimalFormat("#.##");



        /* Drive straight forward at 1/2 of full drive speed */
		double drive_speed = 0.5;


		while ( (runtime.time() < 1.0) && // seconds to run
				!Thread.currentThread().isInterrupted()) {
			if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
				if (yawPIDResult.isOnTarget()) {
					robot.FloatMotors();
					telemetry.addData("PIDOutput", df.format(0.00));
				} else {
					double output = yawPIDResult.getOutput();
					if (output > 0 && output < 0.15) { output = 0.15; }
					if (output < 0 && output > -0.15) { output = -0.15; }
					robot.Drive(output, -output);

					telemetry.addData("PIDOutput", df.format(output) + ", " +
							df.format(-output));
				}
			} else {
			    /* A timeout occurred */
				Log.w("navXRotateOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
			}
			telemetry.addData("Yaw", df.format(navx_device.getYaw()));
		}

		/*


		double right = 0.1;
		double left = 0.1;
		robot.Drive(right,left);

		while (robot.odsSensor.getLightDetected() < 0.2) {
			telemetry.addData("Normal", robot.odsSensor.getLightDetected());
			telemetry.update();
			idle();
		}
		*/

		robot.StopDriving();
		while (1>0) {
			readColor();
			telemetry.addData("red","red = " + colorIsRed());
			telemetry.addData("blue","blue = " + colorIsBlue());
			telemetry.update();
		}

	}

	public void readColor(){
		colorCcache = colorCreader.read(0x04, 1);
	}

	boolean colorIsRed() {
		if ((colorCcache[0] & 0xFF) == 10 ||
				(colorCcache[0] & 0xFF) == 11) {
			return (true);
		}
		else return (false);
	}
	boolean colorIsBlue() {
		if ((colorCcache[0] & 0xFF) == 2 ||
				(colorCcache[0] & 0xFF) == 3) {
			return (true);
		} else return (false);
	}

	public double limit(double a) {
		return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
	}


}