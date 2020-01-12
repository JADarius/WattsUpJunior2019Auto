package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Magura {

    public Motors motors;/*
    public Sensors sensors;
    public Collector collector;*/
    public Servos servos;

    public HardwareMap hardwareMap;

    public Magura (HardwareMap hm) {

       hardwareMap = hm;

    motors= new Motors(
       hm.get(DcMotor.class, Config.left_back),
       hm.get(DcMotor.class, Config.left_front),
       hm.get(DcMotor.class, Config.right_back),
       hm.get(DcMotor.class, Config.right_front)
        );
    /*
    collector= new Collector(
       hm.get(DcMotor.class, Config.brat),
       hm.get(DcMotor.class, Config.mana)
        );

    sensors = new Sensors(
       hm.get(ModernRoboticsTouchSensor.class, Config.rotTouch)
        );*/
    servos = new Servos(
            hm.get(Servo.class, Config.servo_1),
            hm.get(Servo.class, Config.servo_2)
        );
    }

}
