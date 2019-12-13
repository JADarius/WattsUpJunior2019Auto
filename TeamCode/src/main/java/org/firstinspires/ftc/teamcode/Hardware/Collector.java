package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class Collector {

    public DcMotor brat , mana ;

   public Collector (DcMotor rr, DcMotor rl){
      brat=rl;
      mana= rr;

       brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       mana.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

   }

   public void Prindere (double a, double b){
       double power = Range.clip(a + b, -1.0, 1.0);
       mana.setPower(power);
   }

   public void Catapult (double a){
        double power = Range.clip(a, -1.0, 1.0);
        brat.setPower(power);
   }
}
