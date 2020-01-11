package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


public class Motors {

    public DcMotor left_front , right_front ;
    public DcMotor left_back , right_back ;
    public final double masterAngle = Math.PI / 4.0;
    public double face;
    public double angle = 0.0;
    public double mag = 0.0;
    public double lf, lb, rf, rb;


    public double scalePower(final double drivePower) {

       return Math.pow(drivePower, 3);
    }

    public Motors (DcMotor lb, DcMotor lf, DcMotor rb, DcMotor rf){

      left_front= lf;
      right_back= rb;
      right_front= rf;
      left_back= lb;

      face = 0.0;

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setFace(double angle){
        face = angle;
    }

    public void setPower(double modifier){
        left_front.setPower(lf * modifier);
        left_back.setPower(lb * modifier);
        right_back.setPower(rb * modifier);
        right_front.setPower(rf * modifier);
    }

    public void normalize(){
        double maxi = Math.max(Math.max(Math.abs(lf), Math.abs(rb)), Math.max(Math.abs(lb), Math.abs(rf)));
        if(maxi > 1.0)
        {
            lf /= maxi;
            lb /= maxi;
            rf /= maxi;
            rb /= maxi;
        }
    }

    public void drivePower(double turn){
        lf = mag * Math.sin(-angle) - turn;
        rf = mag * Math.cos(-angle) + turn;
        lb = mag * Math.cos(-angle) - turn;
        rb = mag * Math.sin(-angle) + turn;
        normalize();
    }

    public void driveAngle(double x, double y){
        angle = Math.atan2(x,y);
        angle += masterAngle;
        angle += face;
        mag = Math.min(1.0, Math.sqrt(x * x + y * y));
    }

    public void move (double x, double y, double z, double r){
        driveAngle(x ,y);
        drivePower(z);
        setPower(r);
    }

}
