package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Collector {

    public DcMotor rotLeft , rotRight ;

   public Collector (DcMotor rr, DcMotor rl){
      rotLeft=rl;
      rotRight= rr;

       rotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

   }

   public void stopRotation (){
     rotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     rotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

     rotLeft.setPower(0.0);
     rotRight.setPower(0.0);

   }

   public void addPower ( double power){
     rotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
     rotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

     rotRight.setPower(-power);
     rotLeft.setPower(power);

   }

  public void addTicksWithPower (int ticks, double power){

   rotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   rotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

   rotRight.setTargetPosition(Math.abs(rotRight.getCurrentPosition()-ticks));
   rotLeft.setTargetPosition(rotLeft.getCurrentPosition()-ticks);

   rotRight.setPower(power);
   rotLeft.setPower(power);

  }


}
