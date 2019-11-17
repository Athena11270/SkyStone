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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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

    public Servo Towtruck = null;

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
        Towtruck = OpModeReference.hardwareMap.get(Servo.class, "Towtruck");




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
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : RightMotors)
            m.setDirection(DcMotor.Direction.FORWARD);

        // set any properties that apply to ALL motors
        for (DcMotor m : AllMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void mecanumOld (double speed, double strafe, double rotate, boolean fast, boolean slow) {
        double movingSpeed = 1;
        if (fast && !slow)
            movingSpeed = 1;
        else if (slow && !fast)
            movingSpeed = 0.25;
        else
            movingSpeed = 0;

        double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
        double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
        double leftBackDir = Range.clip((-speed + strafe - rotate), -1, 1) * movingSpeed;
        double rightBackDir = Range.clip((-speed - strafe + rotate), -1, 1) * movingSpeed;

        FL.setPower(leftFrontDir);
        FR.setPower(rightFrontDir);
        BL.setPower(leftBackDir);
        BR.setPower(rightBackDir);
        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);

    }

    // just a method to stop driving
    public void stopDriving() {
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    public void driveOld (double inches) {
    FR.setDirection(DcMotorSimple.Direction.REVERSE);
    for (DcMotor m : AllMotors) {
        m.setPower( 0.37);
     }
     OpModeReference.sleep(Math.round(1000*(inches/24)));
     stopDriving();
    }

    public void multiDrive (double forwardblocks, double strafeblocks) {
        double distance = Math.abs(Math.hypot(forwardblocks, strafeblocks));
        double longest;
        double x;
        double y;
        long ms;
        if (Math.abs(forwardblocks) > Math.abs(strafeblocks)) {
            longest = forwardblocks;
            y = 1 * (forwardblocks/Math.abs(forwardblocks));
            x = strafeblocks / longest;
        }
        else {
            longest = strafeblocks;
            x = 1 * (strafeblocks/Math.abs(strafeblocks));
            y = forwardblocks / longest;
        }
        ms = Math.round(distance * 1000);

        x = x * 0.56;
        y = y * 0.37;

        FL.setPower(y + x);
        FR.setPower(y - x);
        BL.setPower(y + x);
        BR.setPower(y - x);

        OpModeReference.sleep(ms);

        stopDriving();
    }

    public void drive (double inches) {
        //WriteTelemetry("Status", "starting drive method...");
        Velocity fast;

        fast = imu.getVelocity();

        fast.toUnit(DistanceUnit.INCH);

        //WriteTelemetry("satus", "about to power motors");
        for (DcMotor m : AllMotors){
            m.setPower(0.37);

        OpModeReference.sleep(100);


        //WriteTelemetry("status", "calculating velocity");
        double vel = Math.sqrt((fast.xVeloc*fast.xVeloc)+(fast.yVeloc*fast.yVeloc));



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

    public void TowtruckControl () {
        Towtruck.setPosition(Range.clip((1-OpModeReference.gamepad1.right_trigger),0,0.5));
        OpModeReference.telemetry.addData("Towtruck Position", Towtruck.getPosition());
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

