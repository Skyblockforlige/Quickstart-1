package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@TeleOp
public class intakewithcolorsensor extends LinearOpMode {
    ControlSystem cs1;
    DcMotorEx spindexer;
    NormalizedColorSensor colorSensor;
    DcMotorEx intake;
    float[] hsv = new float[3];
    int target = 0;
    @Override
    public void runOpMode(){
        rconstants.initHardware(hardwareMap);
        spindexer=rconstants.spindexer;
        colorSensor=rconstants.colorSensor;
        intake=rconstants.intake;
        spindexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        cs1 = ControlSystem.builder()
                .posPid(0.0009, 0, 0)
                .build();
        colorSensor.setGain(3);
        waitForStart();

        if(opModeIsActive()){
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            float hue = hsv[0];
            boolean isPurple = (hue > 0 && hue < 285);
            boolean isGreen  = (hue > 95 && hue < 130);
            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            if(hue>30 && hue<320){
                target+=rconstants.movespindexer;
                sleep(1000);
            }


        }
        while(opModeIsActive()){
            KineticState ks = new KineticState(spindexer.getCurrentPosition(), spindexer.getVelocity());
            cs1.setGoal(new KineticState(target));
            spindexer.setPower(cs1.calculate(ks));
            telemetry.addData("Hue", hsv[0]);
            telemetry.update();
        }
    }
}
