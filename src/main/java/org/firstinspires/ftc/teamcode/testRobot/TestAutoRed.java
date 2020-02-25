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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="TestAutoRed", group="Autonomous")
//@Disabled
public class TestAutoRed extends LinearOpMode
{
    private TestRobot robot = new TestRobot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);



        waitForStart();
        boolean run = true;
        while (opModeIsActive())
        {

            /*
            if (run)
            {
                while (robot.blocksideDistance.getDistance(DistanceUnit.CM) > 30)
                {
                    robot.moveLR(0.55);
                }
                straighten();
                while (robot.blocksideDistance.getDistance(DistanceUnit.CM) > 6.5)
                {
                    robot.moveLR(0.3);
                }
                robot.zeroMotorPower();
                straighten();

                for (int i = 0; i < 4; i++)
                {
                    if (robot.blockColor.green() + robot.blockColor.red() < 800)
                        telemetry.addLine("skytone");
                    else
                        telemetry.addLine("brock");

                    robot.moveFB(0.7, 505);
                    while (robot.motorsAreBusy())
                    {
                        //this waits for the motors to get into position}
                    }
                    //robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES)
                }
                robot.zeroMotorPower();
                if (robot.blockColor.green() + robot.blockColor.red() < 820)
                    telemetry.addLine("skytone");
                else
                    telemetry.addLine("brock");
                run = false;
                telemetry.update();
            }*/
        }
    }



    private void straighten()
    {
        boolean isAlligned = false;

        while(isAlligned) {
            if(!robot.motorsAreBusy()) {
                Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (angles.firstAngle > 0.5)
                {
                    // turn right
                    robot.rotate(0.5, 35);
                }
                else if (angles.firstAngle < -.4)
                {
                    // turn left
                    robot.rotate(0.5, -35);
                    sleep(30);
                }
                else
                {
                    robot.zeroMotorPower();
                    isAlligned = true;
                }
            }
        }

    }

    private boolean isBlack()
    {
        return (robot.blockColor.green() + robot.blockColor.red() < 820);
    }






    private int firstPosition = 0;
    private int secondPosition = 0;
/*
    private void getBlockLayout()
    {
        while((firstPosition < 2) || secondPosition == 0)
        {
            if(isBlack())
            {
                secondPosition = firstPosition + 2;
            }
            else
            {
                firstPosition++;
                nextPos;
            }
        }
    }
*/

}
