package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled

public class Viper {

    double ticksPerViperInch = (537.7 / ((24/25.4) * Math.PI));
    boolean Run = true;
    private DcMotor viper;
    int currentDistance = 0;

    public double viperMovement(int distance) {


        //double currentDistance = viper.getCurrentPosition()/ticksPerViperInch;
        while (Run = true) {

            int movement = (int) (ticksPerViperInch * (distance - currentDistance));
            viper.setTargetPosition(-movement);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperPowerCalc(distance);
            distance = currentDistance;
            break;
        }
        return currentDistance;
    }
    public double viperMovementSetSpeed(int distance, double power) {

        //double currentDistance = viper.getCurrentPosition()/ticksPerViperInch;
        while (Run = true) {

            int movement = (int) (ticksPerViperInch * (distance - currentDistance));
            viper.setTargetPosition(-movement);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viper.setPower(power);
            distance = currentDistance;
            break;
        }
        return currentDistance;
    }
    public void ViperPowerCalc(int CalcDistance) {
        double viperMult = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getDistance() - CalcDistance) > 0.5) { // Allow for a small error margin

            double error = CalcDistance - getDistance(); // Calculate the error

            // Calculate wheel powers based on error
            double viperPower = error * viperMult;
            double Low = 0.2;
            if(viperPower<Low)
                viperPower = Low;
            viper.setPower(viperPower);
        }
    }
    public double getDistance() {
        double armPos;
        armPos = viper.getCurrentPosition()/ticksPerViperInch;
        return armPos;
    }
}
