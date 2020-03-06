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
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="TestAutoRed", group="Autonomous")

public class TestAutoRed extends LinearOpMode
{
    private TestRobot robot = new TestRobot();

    private StonePosition stonePosition;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        if(opModeIsActive())
        {

            while(robot.blocksideDistance.getDistance(DistanceUnit.CM) > 30)
            {
                robot.moveLR(0.55);
            }
            straighten(robot);
            while (robot.blocksideDistance.getDistance(DistanceUnit.CM) > 8)
            {
                robot.moveLR(0.3);
            }
            robot.zeroMotorPower();
            straighten(robot);



            /*
            test the isBlack on both the sensors
            locate the block positions based on a single read from each sensor
             */
            sleep(50);
            if(isBlack(robot.blockColorRight)) // if first block is black
                stonePosition = StonePosition.OneandFour; // block at position 1 and 4

            else if(isBlack(robot.blockColorLeft)) // if third block is black
                stonePosition = StonePosition.ThreeandSix; // block at position 3 and 6

            else // if neither of the blocks are black
                stonePosition = StonePosition.TwoandFive; // block at position 2 and 5

            switch(stonePosition)
            {
                case OneandFour:
                    telemetry.addData("Stone Position", stonePosition);
                    telemetry.update();
                    robot.zeroMotorPower();
                    break;

                case TwoandFive:
                    break;

                case ThreeandSix:
                    break;

                default:
                    break;
            }
        }

    }



    private void straighten(TestRobot robot)
    {
        boolean isAlligned = false;

        while(isAlligned)
        {
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

    private boolean isBlack(ColorSensor sensor)
    {
        return (sensor.green() + sensor.red() < 820);
    }


}
