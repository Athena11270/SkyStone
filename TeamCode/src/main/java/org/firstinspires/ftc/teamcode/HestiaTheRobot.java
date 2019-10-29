package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/**
 * This is NOT an opmode.
 * This is a class that:
 *    - Contains all of your robot hardware
 *    - Does all initialization code
 *    - Has all of the methods for driving and turning
 *
 *    - This example is for the robot named Suspect, built by Astrid and Ari in 45 min
 */



public class HestiaTheRobot
{
    // declare hardware imu, motors, servos, sensors
    BNO055IMU imu;
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    // create arrays for your motors (change sizes to match YOUR number of motors)
    public DcMotor[] LeftMotors = new DcMotor[2];
    public DcMotor[] RightMotors = new DcMotor[2];
    public DcMotor[] AllMotors = new DcMotor[4];

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;

    public HestiaTheRobot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    // This is a method for all of the initialization code - all of the
    // stuff that happens after clicking the Initialize button, but before
    // clicking the start button.
    public void initialize() {

        // this is the IMU crap...just...accept it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        FL = OpModeReference.hardwareMap.get(DcMotor.class, "left_front");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "right_front");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "left_back");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "right_back");
        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "imu");

        // initialize the IMU
        imu.initialize(parameters);


        // now add each motor to your motor arrays (this example only has 2 motors)
        // left
        LeftMotors[0] = FL;
        LeftMotors[1] = BL;
        // right
        RightMotors[0] = FR;
        RightMotors[1] = BR;
        // all
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = BL;
        AllMotors[3] = BR;

        // set the direction for all left, then all right motors
        for (DcMotor m : LeftMotors)
            m.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor m : RightMotors)
            m.setDirection(DcMotor.Direction.FORWARD);

        // set any properties that apply to ALL motors
        for (DcMotor m : AllMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // just a method to stop driving
    public void stopDriving() {
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    public void drive (double inches) {
        WriteTelemetry("Status", "starting drive method...");
        Acceleration gravity;
        WriteTelemetry("Status", "about to get gravity");
        gravity = imu.getGravity();

        WriteTelemetry("satus", "about to power motors");
        for (DcMotor m : AllMotors){
            m.setPower(0.37);

        OpModeReference.sleep(100);

        double meters = inches * 0.0254;
        WriteTelemetry("status", "calculating velocity");
        double vel = Math.sqrt(gravity.xAccel*gravity.xAccel+gravity.yAccel*gravity.yAccel);
        long total = Math.round(vel/meters * 1000);
        WriteTelemetry("Status","did the garbage");
        //WriteTelemetry("sleep", total-500);
        OpModeReference.sleep(total-100);

        stopDriving();

        }

    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    public double getAngleDifference(double startAngle) {

        Orientation angles;
        angles = imu.getAngularOrientation();

        double angleDifference = angles.thirdAngle - startAngle;

        // handle going past the 0 or 180 barriers
        // where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }


    public void turn(double targetAngleDifference) {

        Orientation angles;
        angles = imu.getAngularOrientation();

        double direction = targetAngleDifference/Math.abs(targetAngleDifference*targetAngleDifference);
        double startingAngle = angles.thirdAngle;

        while (getAngleDifference(startingAngle) < Math.abs(targetAngleDifference*0.75)){
            for (DcMotor m : LeftMotors){
                m.setPower(-0.5*direction);
            }
            for (DcMotor m : RightMotors){
                m.setPower(0.5*direction);
            }
        }
        while (getAngleDifference(startingAngle) < Math.abs(targetAngleDifference)){
            for (DcMotor m : LeftMotors){
                m.setPower(-0.25*direction);
            }
            for (DcMotor m : RightMotors){
                m.setPower(0.25*direction);
            }
        }
        stopDriving();
    }

    public void WriteTelemetry(String caption, Object value){
        OpModeReference.telemetry.addData(caption, value);
        OpModeReference.telemetry.update();
    }
}

