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

@TeleOp(name="TeleOp", group="Linear Opmode")

public class OpMode extends LinearOpMode {
    private Magura robot;
    @Override
    public void runOpMode(){
        robot = new Magura(hardwareMap);
        telemetry.addData("Status","Initialized");
        telemetry.update();
        double modifier = 1.0;
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            if(gamepad1.x)
                robot.motors.setFace(-Math.PI / 2.0);
            else if(gamepad1.b)
                robot.motors.setFace(Math.PI / 2.0);
            if(gamepad1.a)
                robot.servos.Apuca();
            else if(gamepad1.y)
                robot.servos.Desprinde();
            if(gamepad1.right_trigger > 0.3)
                modifier = 0.5;
            final double x = robot.motors.scalePower(gamepad1.left_stick_x);
            final double y = robot.motors.scalePower(gamepad1.left_stick_y);
            final double turn = robot.motors.scalePower(gamepad1.right_stick_x);
            robot.motors.move(x, y, turn, modifier);
            /*if(gamepad1.x) {
                if(!x_press)
                {
                    robot.servos.Apuca();
                    x_press = true;
                }
                else
                    x_press = false;
            }
            if(gamepad1.b) {
                if(!b_press)
                {
                    robot.servos.Desprinde();
                    b_press = true;
                }
                else
                    b_press = false;
            }*/
        }
    }
}