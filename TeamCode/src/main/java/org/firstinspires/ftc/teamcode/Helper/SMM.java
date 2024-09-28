package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.Servo;
//smm stands for sample manipulating mechanism
public class SMM {
private CRServoImpl smmW;
private Servo smmT;
//turns wheels to take in samples, servo called smmW
public void smmWMovement(double power){
    smmW.setPower(power);
}

//for turning the smm overall, motor called smmT
public void smmTMovement(double Move){
    smmT.setPosition(Move);
}


















}
