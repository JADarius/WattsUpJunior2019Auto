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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Magura;

@TeleOp

public class OpMode extends LinearOpMode {
    private Magura robot;
    public double scalePower(final double drivePower) {
        return Math.pow(drivePower, 3);
    }
    @Override
    public void runOpMode(){
        boolean x_press = false;
        boolean b_press = false;
        telemetry.addData("Status","Initialized");
        telemetry.update();

        robot.collector.rotLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        double circumferenceSmall = Math.PI * 57.15;
        int Turn = (int)(circumferenceSmall * 2 / 1120);
        double drive = 0.0; //puterea fata-spate
        double turn = 0.0; //puterea stanga-dreapta
        double strafe = 0.0;
        double Speed = 0.0;
        double powerLf = 0.0;
        double powerRf = 0.0;
        double powerLb = 0.0;
        double powerRb = 0.0;
        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            if(gamepad1.right_trigger > 0.0)
                Speed = 0.5;
            else
                Speed = 1;
            powerLf = Range.clip(drive + strafe + turn ,-1.0,1.0);
            powerRf = Range.clip(drive - strafe - turn ,-1.0,1.0);
            powerLb = Range.clip(drive - strafe + turn ,-1.0,1.0);
            powerRb = Range.clip(drive + strafe - turn ,-1.0,1.0);
            robot.motors.left_front.setPower(scalePower(powerLf * Speed));
            robot.motors.right_front.setPower(scalePower(powerRf * Speed));
            robot.motors.left_back.setPower(scalePower(powerLb * Speed));
            robot.motors.right_back.setPower(scalePower(powerRb * Speed));
            double entrancePower = 0.0;
            if(gamepad1.x) {
                if(!x_press)
                {
                    robot.collector.stopRotation();
                    if(entrancePower==0.0)
                    {
                        entrancePower=1;
                        robot.collector.addPower(entrancePower);
                    }
                    else
                    {
                        entrancePower=0;
                        robot.collector.addPower(entrancePower);
                    }
                    x_press = true;
                }
                else
                    x_press = false;
            }
            if(gamepad1.b) {
                if(!b_press)
                {
                    robot.collector.stopRotation();
                    robot.collector.addTicksWithPower(Turn,1);
                    b_press = true;
                }
                else
                    b_press = false;
            }
            if(!robot.collector.rotLeft.isBusy())
                robot.collector.stopRotation();
        }
    }
}