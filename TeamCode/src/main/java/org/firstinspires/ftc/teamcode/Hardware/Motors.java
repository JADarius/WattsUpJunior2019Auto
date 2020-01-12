package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



public class Motors {

    private DcMotor left_front , right_front ;
    private DcMotor left_back , right_back ;
    private final double masterAngle = Math.PI / 4.0;
    private double face;
    private double angle = 0.0;
    private double mag = 0.0;
    private double lf, lb, rf, rb;
    private static final double TICK_COUNT = 560;
    private static final double circumference = Math.PI * 100.0;

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
      right_back.setDirection(DcMotorSimple.Direction.FORWARD);
      right_front.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void setFace(double angle){
        face = angle;
    }

    private void setPower(double modifier){
        left_front.setPower(lf * modifier);
        left_back.setPower(lb * modifier);
        right_back.setPower(rb * modifier);
        right_front.setPower(rf * modifier);
    }

    private void normalize(){
        double maxi = Math.max(Math.max(Math.abs(lf), Math.abs(rb)), Math.max(Math.abs(lb), Math.abs(rf)));
        if(maxi > 1.0)
        {
            lf /= maxi;
            lb /= maxi;
            rf /= maxi;
            rb /= maxi;
        }
    }

    private void drivePower(double turn){
        lf = mag * Math.sin(-angle) - turn;
        rf = mag * Math.cos(-angle) + turn;
        lb = mag * Math.cos(-angle) - turn;
        rb = mag * Math.sin(-angle) + turn;
        normalize();
    }

    private void driveAngle(double x, double y){
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

    public void reset(){
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setPowerA(double speed){
        left_back.setPower(speed);
        left_front.setPower(speed);
        right_back.setPower(speed);
        right_front.setPower(speed);
    }

    private void brake(){
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    private void runToPosition(int lf, int lb, int rf, int rb){
        left_front.setTargetPosition(lf);
        left_back.setTargetPosition(lb);
        right_front.setTargetPosition(rf);
        right_back.setTargetPosition(rb);

        left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveFB(double speed, double distance){
        reset();
        int target = (int)(distance * TICK_COUNT / circumference);
        runToPosition(target,target,target,target);
        setPowerA(speed);
        while (left_back.isBusy() && left_front.isBusy()){

        }
        brake();
    }

    public void turn(double speed, double angle){
        reset();
        int  target = (int) (angle * circumference / 360);
        runToPosition(target,target,target,target);
        setPowerA(speed);
        while (left_back.isBusy() && right_back.isBusy()){

        }
        brake();
    }

    public void driveLR(double speed, double distance){
        reset();
        int target = (int)(distance * TICK_COUNT / circumference);
        runToPosition(target,-target,-target,target);
        setPowerA(speed);
        while(left_back.isBusy() && right_back.isBusy()){

        }
    }
}
