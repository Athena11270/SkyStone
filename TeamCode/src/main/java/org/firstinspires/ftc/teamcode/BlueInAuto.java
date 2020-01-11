package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "BlueIn_Foundation", group = "IMU1")
public class BlueInAuto extends LinearOpMode {


    @Override
    public void runOpMode() {

        // create an instance of the hardware robot class, initialize, pass an instance of THIS OpMode
        HestiaTheRobot hestia = new HestiaTheRobot(this);
        hestia.initialize();

        waitForStart();

        // move an inch away from the wall to reduce drag
        hestia.drive(-1, 0.25);

        // move to in front of the tray
        hestia.strafeRight(30, 0.25);

        // get closer to the tray
        hestia.drive(-36, 0.25);

        // put down the towing arms, wait for them to get down
        hestia.towtruck(false);
        sleep(1000);

        // drag tray to building site
        hestia.drive(48, 0.25);

        // raise towing arms (and wait)
        hestia.towtruck(true);
        sleep(1000);

        // move toward the line
        hestia.strafeLeft(30, 0.25);

        // move toward inside to avoid other robot
        hestia.drive(-24, 0.25);

        // park over line
        hestia.strafeLeft(24, 0.25);
    }
}