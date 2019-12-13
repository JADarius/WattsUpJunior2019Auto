package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Servos {

    public Servo servo_1, servo_2;

    public Servos (Servo s1, Servo s2){
        servo_1 = s1;
        servo_2 = s2;
    }
    public void Apuca (){
        servo_1.setPosition(1.0);
        servo_2.setPosition(1.0);
    }
    public void Desprinde (){
        servo_1.setPosition(0.0);
        servo_2.setPosition(0.0);
    }
}
