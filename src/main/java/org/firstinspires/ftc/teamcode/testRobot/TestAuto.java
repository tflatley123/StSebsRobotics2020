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


@TeleOp(name="Testing", group="Autonomous")
//@Disabled
public class TestAuto extends LinearOpMode
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
            if(gamepad1.dpad_down)
            {
                robot.linear1.setPower(-1);
                robot.linear2.setPower(-1);
            }
            else if(gamepad1.dpad_up)
            {
                robot.linear1.setPower(1);
                robot.linear2.setPower(1);
            }
            else
            {
                robot.linear1.setPower(0);
                robot.linear2.setPower(0);
            }
            /*// bring it 18 inches from row of blocks (46cm)
            if(gamepad1.dpad_up)
                power += 0.1;
            if(gamepad1.dpad_down)
                power -= 0.1;
            if(gamepad1.dpad_right)
                moveTicks -= 10;
            if(gamepad1.dpad_left)
                moveTicks += 10;
            if(gamepad1.a)
                move = !move;


            addedColor = robot.blockColor.green() + robot.blockColor.red();


            // 800 is r + g values with the light on
            // this is accurate when the bot is btw 7.5 and 4.5 cm away

            if(addedColor < 800)
            {
                telemetry.addLine("SkyStone");
            }
            telemetry.addData("Power", power);
            telemetry.addData("Ticks", moveTicks);

            telemetry.addLine();

            telemetry.addData("R2", robot.blockColor.red());
            telemetry.addData("B2", robot.blockColor.blue());
            telemetry.addData("G2", robot.blockColor.green());

            telemetry.addLine();

            telemetry.addData("Orientation: ", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES));
            telemetry.addData("Distance to Block: ", robot.blocksideDistance.getDistance(DistanceUnit.CM));

            if(move)
            {
                robot.moveFB(power, moveTicks);
                move = false;
            }
            */

            telemetry.addData("Distance to Block: ", robot.blocksideDistance.getDistance(DistanceUnit.CM));
            telemetry.addData("R2", robot.blockColor.red());
            telemetry.addData("B2", robot.blockColor.blue());
            telemetry.addData("G2", robot.blockColor.green());

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
