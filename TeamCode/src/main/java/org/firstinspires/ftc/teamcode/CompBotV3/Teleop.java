package org.firstinspires.ftc.teamcode.CompBotV3;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBot.CompBotHW;

@TeleOp
public class Teleop extends OpMode {
    CompBotV3 r = new CompBotV3();
    double initialHeading;
    boolean headingReset = false;

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y, x = gamepad1.left_stick_x, turn = gamepad1.right_stick_x;

        if (Math.abs(turn) < 0.1) {
            if(!headingReset) {
                initialHeading = r.imu.getHeading();
                headingReset = true;
            } else {
                double error = r.imu.getHeading() - initialHeading;
                r.fl.setPower(MathUtils.clamp(-(y + x) + CompBotV3.corrCoeff*error,-1,1));
                r.fr.setPower(MathUtils.clamp(y - x - CompBotV3.corrCoeff*error,-1,1));
                r.bl.setPower(MathUtils.clamp(-(y - x) + CompBotV3.corrCoeff*error,-1,1));
                r.br.setPower(MathUtils.clamp(y + x - CompBotV3.corrCoeff*error,-1,1));
            }
        } else {
            r.fl.setPower(MathUtils.clamp(-(y + x) + turn,-1,1));
            r.fr.setPower(MathUtils.clamp(y - x - turn,-1,1));
            r.bl.setPower(MathUtils.clamp(-(y - x) + turn,-1,1));
            r.br.setPower(MathUtils.clamp(y + x - turn,-1,1));
        }

    }
    @Override
    public void stop() {
        r.stop();
    }
}
