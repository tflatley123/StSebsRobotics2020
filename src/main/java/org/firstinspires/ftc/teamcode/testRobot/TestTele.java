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

package org.firstinspires.ftc.teamcode.testRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.TFOD;


@TeleOp(name="Testing", group="TeleOp")
//@Disabled
public class TestTele extends LinearOpMode
{
    private TestRobot robot = new TestRobot();

    @Override
    public void runOpMode()
    {
        boolean move = false;
        int moveTicks = 0;
        double power = 0;

        int addedColor = 0;

        robot.init(hardwareMap);


        waitForStart();

        while (opModeIsActive())
        {
            robot.backleftPower = gamepad1.left_stick_y;
            robot.frontleftPower = gamepad1.left_stick_y;
            robot.frontrightPower = gamepad1.right_stick_y;
            robot.backrightPower = gamepad1.right_stick_y;

            if((gamepad1.dpad_right || gamepad1.dpad_left) || (gamepad1.dpad_up || gamepad1.dpad_down))
            {
                if (gamepad1.dpad_left) // left
                {
                    robot.moveFB(0.6);

                } else if (gamepad1.dpad_right) // right
                {
                    // move backwards
                    robot.moveFB(-0.6);
                }

                if (gamepad1.dpad_down) // down
                {
                    // move right
                    robot.moveLR(-0.7);

                }
                else if (gamepad1.dpad_up) // forward
                {
                    // move left
                    robot.moveLR(0.7);
                }

                if (gamepad1.dpad_right && gamepad1.dpad_up) // up right
                {
                    // up left
                    robot.backleftPower = -0.8;
                    robot.frontleftPower = 0.15;
                    robot.frontrightPower = -0.8;
                    robot.backrightPower = 0.15;
                }
                else if (gamepad1.dpad_right && gamepad1.dpad_down) // down right
                {
                    // up right
                    robot.backleftPower = 0.15;
                    robot.frontleftPower = -0.8;
                    robot.frontrightPower = 0.15;
                    robot.backrightPower = -0.8;

                } else if (gamepad1.dpad_left && gamepad1.dpad_up) // up left
                {

                    // down left
                    robot.backleftPower = -0.15;
                    robot.frontleftPower = 0.8;
                    robot.frontrightPower = -0.15;
                    robot.backrightPower = 0.8;

                }
                else if (gamepad1.dpad_left && gamepad1.dpad_down) // down left
                {

                    // down right
                    robot.backleftPower = 0.8;
                    robot.frontleftPower = -0.15;
                    robot.frontrightPower = 0.8;
                    robot.backrightPower = -0.15;
                }
            }
            if(gamepad2.dpad_down)
            {
                robot.linearSlide.setPower(1);
            }
            else if(gamepad2.dpad_up)
            {
                robot.linearSlide.setPower(-1);
            }
            else
            {
                robot.linearSlide.setPower(0);
            }
            if(gamepad2.dpad_right)
                robot.armServo.setPower(-1);
            else if(gamepad2.dpad_left)
                robot.armServo.setPower(1);
            else
                robot.armServo.setPower(0);


            telemetry.addData("Distance to Block: ", robot.blocksideDistance.getDistance(DistanceUnit.CM));
            telemetry.addLine("RIGHT");
            telemetry.addData("R", robot.blockColorRight.red());
            telemetry.addData("B", robot.blockColorRight.blue());
            telemetry.addData("G", robot.blockColorRight.green());
            telemetry.addLine("LEFT");
            telemetry.addData("R", robot.blockColorLeft.red());
            telemetry.addData("B", robot.blockColorLeft.blue());
            telemetry.addData("G", robot.blockColorLeft.green());

            telemetry.update();

        }

    }
    private void keepStraight()
    {
        boolean isAlligned = false;

        while(isAlligned) {
            if(!robot.motorsAreBusy()) {
                Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (angles.firstAngle > 0.5) {
                    // turn right
                    robot.rotate(0.6, 35);
                } else if (angles.firstAngle < -.4) {
                    // turn left
                    robot.rotate(0.3, -35);
                    sleep(30);
                } else
                {
                    isAlligned = true;
                }
            }
        }

    }
}
