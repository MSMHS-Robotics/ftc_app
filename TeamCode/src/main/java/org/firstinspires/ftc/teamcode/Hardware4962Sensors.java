package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * Created  on 10/9/16.
 */

public class Hardware4962Sensors extends Hardware4962 {
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
    private final double YAW_PID_P = 0.01;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private boolean calibration_complete = false;



    public void initSensors(HardwareMap ahwMap) {
        hwMap = ahwMap;
        odsSensor = hwMap.opticalDistanceSensor.get("ods");
        colorC = hwMap.i2cDevice.get("color sensor");
        colorCreader = new I2cDeviceSynchImpl(colorC, new I2cAddr(0x1e), false);
        colorCreader.engage();

        colorCreader.write8(3, 1);    //Set the mode of the color sensor to passive





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


}
