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

    public void move (double drive, double turn, double speed ){

        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);

        right_front.setPower(scalePower(rightPower * speed));
        right_back.setPower(scalePower(rightPower * speed));

        left_front.setPower(scalePower(leftPower * speed));
        left_back.setPower(scalePower(leftPower * speed));

    }





}
