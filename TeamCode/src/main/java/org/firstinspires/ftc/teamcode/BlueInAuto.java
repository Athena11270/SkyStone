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

        // create an instance of the hardware robot class, pass an instance of THIS OpMode
        HestiaTheRobot hestia = new HestiaTheRobot(this);

        hestia.initialize();

        waitForStart();

        hestia.strafeRight(24, 0.25);

        hestia.drive(-24, 0.25);

        hestia.towtruck(false);

        sleep(1000);

        hestia.drive(48, 0.25);

        hestia.towtruck(true);

        sleep(1000);

        hestia.strafeLeft(52, 0.25);
    }
}