package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "RedInWithVision", group = "IMU1")
public class RedInWithVision extends LinearOpMode {


    @Override
    public void runOpMode() {

        // create an instance of the hardware robot class, pass an instance of THIS OpMode
        HestiaTheRobot hestia = new HestiaTheRobot(this);

        double speed = 0.35;

        hestia.initialize();
        hestia.CameraStart();

        waitForStart();

        hestia.strafeLeft(12,speed);

        hestia.drive(-2, 0.25);

        int blockPos = hestia.RedFindSkystone();

        hestia.strafeLeft(10, speed);

        if (blockPos == 1)

            hestia.drive(-8, speed);

        else if (blockPos == 2)

            hestia.drive(-2, speed);

        else

            hestia.drive(6, speed);

        hestia.strafeLeft(14, speed);

        //makes sure it has block

        hestia.SideHuggerControl(1);

        hestia.drive(2, 0.25);

        hestia.SideHuggerControl(1);

        hestia.drive(-4, 0.25);

        hestia.SideHuggerControl(1);

        hestia.drive(2, 0.25);

        //moves away from blocks so it doesnt hit bridge

        hestia.strafeRight(13, speed);

        //drives through bridge

        hestia.drive(60 - (blockPos * 8), 0.7);

        hestia.drive(8, speed);

        //sets down block

        hestia.SideHuggerControl(0.5);

        hestia.strafeLeft(10, 0.25);

        hestia.strafeRight(7, 0.25);

        //her arm can't reach the block when it's [X][ ][ ], so this just makes her go for the middle one

        if (blockPos == 1)

            blockPos += 1;

        //grabs other block

        hestia.drive(-(92 - (blockPos * 8)), 0.7);

        hestia.strafeLeft(13, speed);

        //makes sure it has block again

        hestia.SideHuggerControl(1);

        hestia.drive(2, 0.25);

        hestia.SideHuggerControl(1);

        hestia.drive(-4, 0.25);

        hestia.SideHuggerControl(1);

        hestia.drive(2, 0.25);

        //goes to other side again

        hestia.strafeRight(11, speed);

        hestia.drive(92 - (blockPos * 8), 0.7);

        //sets down block

        hestia.SideHuggerControl(0.5);

        hestia.strafeLeft(6, speed);

        hestia.strafeRight(6, speed);

        //gets over line

        hestia.drive(-18, 0.4);

        hestia.strafeLeft(6, 0.25);

    }
}