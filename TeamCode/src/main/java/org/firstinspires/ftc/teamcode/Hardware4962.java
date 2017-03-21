/**
 * Hardware initialization for 4962 robot
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 */

/* adb setup.  Connect phone to USB. Connect computer wifi to Wifi Direct. In terminal type:
1) adb usb
2) adb tcpip 5555
3) adb connect 192.168.49.1
4) (unplug phone)
5) adb connect 192.168.49.1
 */
public class Hardware4962 {

    /* Public OpMode members. */
    public DcMotor rightfrontMotor = null;
    public DcMotor leftfrontMotor = null;
    public DcMotor rightbackMotor = null;
    public DcMotor leftbackMotor = null;
    public DcMotor intakeMotor = null;
    public DcMotor elevatorMotor = null;
    public DcMotor frontshooterMotor = null;
    public DcMotor backshooterMotor = null;
    public Servo button = null;
    public Servo kicker = null;
    public Servo launch = null;
    public Servo wallfront = null;
    public Servo wallback = null;


    public OpticalDistanceSensor odsSensor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware4962() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        rightfrontMotor = hwMap.dcMotor.get("motor right front");
        leftfrontMotor = hwMap.dcMotor.get("motor left front");
        rightbackMotor = hwMap.dcMotor.get("motor right back");
        leftbackMotor = hwMap.dcMotor.get("motor left back");
        intakeMotor = hwMap.dcMotor.get("motor intake");
        elevatorMotor = hwMap.dcMotor.get("motor elevator");
        frontshooterMotor = hwMap.dcMotor.get("shooter front");
        backshooterMotor = hwMap.dcMotor.get("shooter back");
        odsSensor = hwMap.opticalDistanceSensor.get("ods");
        button = hwMap.servo.get("blue");
        kicker = hwMap.servo.get("kicker");
        launch = hwMap.servo.get("launch");
        wallfront = hwMap.servo.get("wallfront");
        wallback = hwMap.servo.get("wallback");

        rightfrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightbackMotor.setDirection(DcMotor.Direction.REVERSE);
        backshooterMotor.setDirection(DcMotor.Direction.REVERSE);
        frontshooterMotor.setMaxSpeed(3000);
        backshooterMotor.setMaxSpeed(3000);
        frontshooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backshooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        button.setPosition(0); //  110/255 is out
        kicker.setPosition(0.59);
        //kicker.setPosition(0);
        launch.setPosition(0.31);
        wallfront.setPosition(0);
        wallback.setPosition(0.6);


        // Set all motors to zero power

        StopDriving();

        //Set all motors to run without encoders.
        //May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Define and initialize ALL installed servos.
        //leftClaw = hwMap.servo.get("left_hand");
        //rightClaw = hwMap.servo.get("right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }

    public void StopDriving() {
        //telemetry.addData("Stopping, Enc:", motorLeft1.getCurrentPosition());
        Drive(0, 0);
    }

    public void Drive(double left, double right) {
        leftfrontMotor.setPower(left);
        leftbackMotor.setPower(left);
        rightfrontMotor.setPower(right);
        rightbackMotor.setPower(right);
    }
    public void FloatMotors() {
        leftfrontMotor.setPowerFloat();
        leftbackMotor.setPowerFloat();
        rightfrontMotor.setPowerFloat();
        rightbackMotor.setPowerFloat();
    }
    public void ShooterSpeed(double speed) {
        frontshooterMotor.setPower(speed);
        backshooterMotor.setPower(speed);
    }
    public void ShooterSpeed(double speedfront, double speedback) {
        frontshooterMotor.setPower(speedfront);
        backshooterMotor.setPower(speedback);
    }


}