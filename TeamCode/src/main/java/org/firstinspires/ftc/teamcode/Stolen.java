/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="StolenFromHiccopampus", group="Linear Opmode")
public class Stolen extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double motionspeed = 0.5;

            if (gamepad1.left_trigger > 0.1 & gamepad1.right_trigger < 0.1)
                motionspeed = 0.1;
            else if (gamepad1.right_trigger > 0.1 & gamepad1.left_trigger < 0.1)
                motionspeed = 0.75;
            else if (gamepad1.right_trigger > 0.1 & gamepad1.left_trigger > 0.1)
                motionspeed = 1;
            else
                motionspeed = 0.5;

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX * motionspeed;
            final double v2 = r * Math.sin(robotAngle) - rightX * motionspeed;
            final double v3 = r * Math.sin(robotAngle) + rightX * motionspeed;
            final double v4 = r * Math.cos(robotAngle) - rightX * motionspeed;

            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftBack.setPower(v3);
            rightBack.setPower(v4);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }
    }
}


