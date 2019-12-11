package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


public class Motors {

    public DcMotor left_front , right_front ;
    public DcMotor left_back , right_back ;

    public double scalePower(final double drivePower) {

        return Math.pow(drivePower, 3);
    }

    public Motors (DcMotor lb, DcMotor lf, DcMotor rb, DcMotor rf){

      left_front= lf;
      right_back= rb;
      right_front= rf;
      left_back= lb;

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void move (double angle, double turn, double mag){

        double pow1 = Math.sin(angle + Math.PI/4) * mag;
        double pow2 = Math.sin(angle - Math.PI/4) * mag;
        pow1 = Range.clip(pow1 + turn, -1.0, 1.0);
        pow2 = Range.clip(pow2 + turn, -1.0, 1.0);
        left_front.setPower(scalePower(pow1));
        right_front.setPower(scalePower(pow2));
        left_back.setPower(scalePower(pow2));
        right_back.setPower(scalePower(pow1));

    }




}
