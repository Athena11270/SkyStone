package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "LeftOut_NoFoundation", group = "IMU1")
public class LeftOutAuto extends LinearOpMode {


    @Override
    public void runOpMode() {

        // create an instance of the hardware robot class, pass an instance of THIS OpMode
        HestiaTheRobot hestia = new HestiaTheRobot(this);

        hestia.initialize();

        waitForStart();

        hestia.strafeRight(24, 0.25);

    }
}