package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@TeleOp
@Disabled
public class intakewithcolorsensor extends LinearOpMode {

    ControlSystem cs1;
    DcMotorEx spindexer;
    NormalizedColorSensor colorSensor;
    DcMotorEx intake;

    float[] hsv = new float[3];

    int target = 0;
    int ballCount = 0;

    boolean colorPreviouslyDetected = false;

    @Override
    public void runOpMode() {
        rconstants.initHardware(hardwareMap);
        spindexer = rconstants.spindexer;
        colorSensor = rconstants.colorSensor;
        intake = rconstants.intake;

        spindexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        cs1 = ControlSystem.builder()
                .posPid(0.0009, 0, 0)
                .build();

        colorSensor.setGain(3);

        waitForStart();

        while (opModeIsActive()) {

            // Intake control
            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            // Only allow spindexer movement if intake is running
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.05;

            // Read color
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            float hue = hsv[0];
            boolean isPurple = (hue > 260 && hue < 300);  // adjust if needed
            boolean isGreen  = (hue > 95 && hue < 130);

            boolean colorDetected = (isPurple || isGreen);

            // Only move if:
            // 1. intake is running
            // 2. color is detected
            // 3. ball count < 3
            // 4. this is a NEW detection
            if (intakeRunning && colorDetected && !colorPreviouslyDetected && ballCount < 3) {
                sleep(100);
                target += rconstants.movespindexer;  // always move forward
                ballCount++;
                colorPreviouslyDetected = true;
            }

            // Reset detection state once color is gone
            if (!colorDetected) {
                colorPreviouslyDetected = false;
            }

            // PID control of spindexer
            KineticState ks = new KineticState(spindexer.getCurrentPosition(), spindexer.getVelocity());
            cs1.setGoal(new KineticState(target));
            spindexer.setPower(cs1.calculate(ks));

            telemetry.addData("Hue", hue);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Target", target);
            telemetry.addData("Intake Running", intakeRunning);
            telemetry.addData("Detected Purple", isPurple);
            telemetry.addData("Detected Green", isGreen);
            telemetry.update();
        }
    }
}
