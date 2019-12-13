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

    public DcMotor SL = null;
    public DcMotor SR = null;

    public Servo RTowtruck = null;
    public Servo LTowtruck = null;

    // create arrays for your motors (change sizes to match YOUR number of motors)
    public DcMotor[] LeftMotors = new DcMotor[2];
    public DcMotor[] RightMotors = new DcMotor[2];
    public DcMotor[] AllMotors = new DcMotor[4];
    public Servo[] TowTruck = new Servo[2];

    // define and calculate constants...
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;    // REV Hex HD 20:1
    static final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
    static final double     WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV  / WHEEL_CIRCUMFERENCE_INCHES);

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
        LTowtruck = OpModeReference.hardwareMap.get(Servo.class, "LTow");
        RTowtruck = OpModeReference.hardwareMap.get(Servo.class, "RTow");
        SL = OpModeReference.hardwareMap.get(DcMotor.class, "LeftSlurp");
        SR = OpModeReference.hardwareMap.get(DcMotor.class, "RightSlurp");

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
        // towtruck
        TowTruck[0] = RTowtruck;
        TowTruck[1] = LTowtruck;

        // set the direction for all left, then all right motors
        for (DcMotor m : LeftMotors)
            m.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor m : RightMotors)
            m.setDirection(DcMotor.Direction.FORWARD);

        SL.setDirection(CRServo.Direction.REVERSE);
        SR.setDirection(CRServo.Direction.FORWARD);

        RTowtruck.setDirection(Servo.Direction.FORWARD);
        LTowtruck.setDirection(Servo.Direction.REVERSE);

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

    public void servoTest() {
        for (Servo s : TowTruck){
            s.setPosition(-1);
        }
        OpModeReference.sleep(1000);
        for (Servo s : TowTruck){
            s.setPosition(1);
        }
        OpModeReference.sleep(2000);

//        OpModeReference.telemetry.addData("right", RTowtruck.getPosition());
//        OpModeReference.telemetry.addData("left", LTowtruck.getPosition());
    }

    // this is a drive method - takes speed and inches
    // WARNING: YOU WILL NEED TO IMPLEMENT REVERSE
    public void drive(double inches, double speed) {
        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            int targetTicks = (int) (-inches * COUNTS_PER_INCH);

            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION
            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed/2);

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("right current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("left current", FL.getCurrentPosition());
                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void strafeRight(double inches, double speed) {
        strafe(inches, speed);
    }

    public void strafeLeft(double inches, double speed) {
        strafe(-inches, speed);
    }
    //THIS ARE IS AN STRAFING METHOD
    private void strafe(double inches, double speed) {
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            int targetTicks = (int) (COUNTS_PER_INCH * inches * (-12/11));

            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION
            for(DcMotor m : AllMotors) {

                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            FL.setTargetPosition(targetTicks);
            FR.setTargetPosition(-targetTicks);
            BL.setTargetPosition(-targetTicks);
            BR.setTargetPosition(targetTicks);

            // turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed/Math.sqrt(2));


            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
//                OpModeReference.telemetry.addData("right current", FR.getCurrentPosition());
//                OpModeReference.telemetry.addData("left current", FL.getCurrentPosition());
                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void towtruck(boolean up){
        if (up)
            for (Servo s : TowTruck)
                s.setPosition(-1);
        else
            for (Servo s : TowTruck)
                s.setPosition(1);
    }


    // This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
    public void turn(double targetAngleDifference, double power) {

        // before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        // just some boolean variables to tell if we've stepped motor power down
        // might actually want more than two steps
        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        // if target angle is Negative, we're turning RIGHT
        if (targetAngleDifference < 0) {
            // turning right, so we want all right motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(power/2);
            for (DcMotor m : LeftMotors)
                m.setPower(-power/2);
            // sleep a tenth of a second
            // WARNING - not sure why this is needed - but sometimes right turns didn't work without
            OpModeReference.sleep(100);

            // we're turning right, so our target angle difference will be negative (ex: -90)
            // so GetAngleDifference will go from 0 to -90
            // keep turning while difference is greater than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power/4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power/2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/2);
                    firstStepDownComplete = true;
                }

                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorPower", FL.getPower());
                OpModeReference.telemetry.addData("RightMotorPower", FR.getPower());
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {
            // turning left so want all left motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(-power);
            for (DcMotor m : LeftMotors)
                m.setPower(power);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power/4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power/2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/2);
                    firstStepDownComplete = true;
                }
                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorPower", FL.getPower());
                OpModeReference.telemetry.addData("RightMotorPower", FR.getPower());
                OpModeReference.telemetry.update();
            }
        } else {
            // is zero - not turning - just return
            return;
        }

        // turn all motors off
        stopDriving();
    }

    // this is a method to get the current heading/z angle from the IMU
    // WE WANT THE Z ANGLE :)
    // AxesOrder.XYZ means we want thirdAngle
    // AxesOrder.ZYX would mean we want firstAngle
    public double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    public double GetAngleDifference(double startAngle) {
        double angleDifference = GetCurrentZAngle() - startAngle;

        // handle going past the 0 or 180 barriers
        // where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }

    // future method to drive more better with curving, yo.
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

    /// ********************** TELE-OP Methods ************************************

    public void mecanum () {


        double speed = OpModeReference.gamepad1.left_stick_y / Math.sqrt(2);
        double strafe = OpModeReference.gamepad1.left_stick_x;
        double rotate = OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper) {
            movingSpeed = 0.4;
        }
        else {
            movingSpeed = 0.8;
        }

        double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
        double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
        double leftBackDir = Range.clip((speed + strafe - rotate), -1, 1) * movingSpeed;
        double rightBackDir = Range.clip((speed - strafe + rotate), -1, 1) * movingSpeed;

        FL.setPower(leftFrontDir);
        FR.setPower(rightFrontDir);
        BL.setPower(leftBackDir);
        BR.setPower(rightBackDir);

//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

    public void TowtruckControl (boolean twoDrivers) {
        double TTpos;
            if (twoDrivers) {
                if (OpModeReference.gamepad2.right_bumper)
                    TTpos = 1;
                else
                    TTpos = -1;
            }
            else {
                if (OpModeReference.gamepad1.right_bumper)
                    TTpos = 1;
                else
                    TTpos = -1;
            }
        LTowtruck.setPosition(TTpos);
        RTowtruck.setPosition(TTpos);
//        OpModeReference.telemetry.addData("Towtruck Position", Towtruck.getPosition());
    }

    public void SlurpyIntake(boolean twoDrivers) {
        double power = 0;
        if (twoDrivers){
            if (OpModeReference.gamepad2.right_trigger > 0.1 && OpModeReference.gamepad2.left_trigger < 0.1) {
                power = 0.5;
            }
            else if (OpModeReference.gamepad2.right_trigger < 0.1 && OpModeReference.gamepad2.left_trigger > 0.1) {
                power = -0.5;
            }
            else {
                power = 0;
            }
        }
        else {
            if (OpModeReference.gamepad1.right_trigger > 0.1 && OpModeReference.gamepad1.left_trigger < 0.1) {
                power = 0.5;
            }
            else if (OpModeReference.gamepad1.right_trigger < 0.1 && OpModeReference.gamepad1.left_trigger > 0.1) {
                power = -0.5;
            }
            else {
                power = 0;
            }
        }
        SL.setPower(power);
        SR.setPower(power);
    }
}

