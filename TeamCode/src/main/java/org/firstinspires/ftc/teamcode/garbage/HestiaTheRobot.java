package org.firstinspires.ftc.teamcode.garbage;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/**
 * This is NOT an opmode.
 * This is a class that:
 *    - Contains all of your robot hardware
 *    - Does all initialization code
 *    - Has all of the methods for driving and turning
 *
 *    - This example is for the robot named Suspect, built by Astrid and Ari in 45 min
 */

@Disabled

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
//
//    //Blinkin stuff.  DO NOT TOUCH!!!
//    RevBlinkinLedDriver blinkinLedDriver;
//    RevBlinkinLedDriver.BlinkinPattern pattern;
//    Telemetry.Item patternName;
//    Telemetry.Item display;
//    SampleRevBlinkinLedDriver.DisplayKind displayKind;
//    Deadline ledCycleDeadline;
//    Deadline gamepadRateLimit;

    // define and calculate constants...
//    static final double     COUNTS_PER_MOTOR    = 537.6 ;    // REV Hex HD 20:1
//    static final double     WHEEL_DIAMETER_INCHES   = 3.93701;     // For figuring circumference
//    static final double     WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR / WHEEL_CIRCUMFERENCE_INCHES);

    // this is the CONSTRUCTOR for this class
    // from your OpMode, you'll have to pass a reference to the OpMode as the parameter
    // Will look like this:
    //      SuspectTheRobot robot = new SuspectTheRobot(this);
    public HestiaTheRobot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    // This is a method for all of the initialization code - all of the
    // stuff that happens after clicking the Initialize button, but before
    // clicking the start button.
    public void Initialize() {

        // this is the IMU crap...just...accept it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        blinkinLedDriver = OpModeReference.hardwareMap.get(RevBlinkinLedDriver.class, "Blinko");
//        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON;
//        blinkinLedDriver.setPattern(pattern);
//        blinkinLedDriver = OpModeReference.hardwareMap.get(RevBlinkinLedDriver.class, "Blinko");
        // get all your hardware from the hardware map
        // defined in the config on your robot controller phone.
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "front_left");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "front_right");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "back_left");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "back_right");
        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "imu");

        // initialize the IMU
        imu.initialize(parameters);

        Orientation gravity;
        // IMU acceleration garbage.
        //gravity = imu.getGravity();

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
    public void StopDriving() {
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    //    public void ColorGood(){
//        if (OpModeReference.gamepad2.b == true) {
//            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
//        }
//        else if (OpModeReference.gamepad2.right_bumper == true){
//            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
//        }
//        else{
//            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
//        }
//    }

    public void drive (double inches, double speed) {

        for (DcMotor m : AllMotors){
            m.setPower(speed);

        }

    }

    public void mecanum (double speed, double lateral, double rotate, boolean slowcondition, boolean fastcondition){
        //for
    }
    public void tankdrive (double leftspeed, double rightspeed, boolean slowcondition, boolean fastcondition) {
        if (fastcondition) {
            for (DcMotor l : LeftMotors)
                l.setPower(leftspeed);
            for (DcMotor r : RightMotors)
                r.setPower(rightspeed);
        }
        else if (slowcondition) {
            for (DcMotor l : LeftMotors)
                l.setPower(0.15 * leftspeed);
            for (DcMotor r : RightMotors)
                r.setPower(0.15 * rightspeed);
        }
        else {
            for (DcMotor l : LeftMotors)
                l.setPower(0.5 * leftspeed);
            for (DcMotor r : RightMotors)
                r.setPower(0.5 * rightspeed);
        }
    }

    // this is a drive method - takes speed and inches
    // WARNING: YOU WILL NEED TO IMPLEMENT REVERSE
//    public void drive(double speed, double inches) {
//        // Ensure that the opmode is still active
//        if (OpModeReference.opModeIsActive()) {
//
//            // calculate the number of ticks you want to travel (cast to integer)
//            int targetTicks = (int) (inches * COUNTS_PER_INCH);
//
//            // reset ticks to 0 on all motors
//            for (DcMotor m : AllMotors)
//                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            // set target position on all motors
//            // mode must be changed to RUN_TO_POSITION
//            for(DcMotor m : AllMotors) {
//                m.setTargetPosition(targetTicks);
//                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            // turn all motors on!
//            for (DcMotor m : LeftMotors)
//                m.setPower(speed/2);
//            for (DcMotor m : RightMotors)
//                m.setPower(speed/2);
//
//            // just keep looping while both motors are busy
//            // stop if driver station stop button pushed
//            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
//                OpModeReference.telemetry.addData("target ticks", targetTicks);
//                OpModeReference.telemetry.addData("right current", FR.getCurrentPosition());
//                OpModeReference.telemetry.addData("left current", FL.getCurrentPosition());
//                OpModeReference.telemetry.update();
//            }
//
//            // once all motors get to where they need to be, turn them off
//            StopDriving();
//
//            // set motors back to RUN_USING_ENCODERS
//            for (DcMotor m : AllMotors)
//                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }

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
                m.setPower(-power/2);
            for (DcMotor m : LeftMotors)
                m.setPower(power/2);
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
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {
            // turning left so want all left motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(power);
            for (DcMotor m : LeftMotors)
                m.setPower(-power);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

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
        } else {
            // is zero - not turning - just return
            return;
        }

        // turn all motors off
        StopDriving();
    }
}

