package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp
public class spindexerwonecolorsensor extends LinearOpMode {

    ControlSystem cs1;
    DcMotorEx spindexer;
    NormalizedColorSensor colorSensor;

    float[] hsv = new float[3];

    static final int ROT = 8192;
    static final int SLOT_TICKS = 2731;

    // 0 = none, 1 = purple, 2 = green
    int[] slotType = new int[3];

    int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        rconstants.initHardware(hardwareMap);
        spindexer = rconstants.spindexer;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "cs");

        spindexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        cs1 = ControlSystem.builder()
                .posPid(0.0009, 0, 0)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            int currPos = spindexer.getCurrentPosition();
            int currMod = floorMod(currPos, ROT);
            int currSlot = currMod / SLOT_TICKS; // 0,1,2

            // Read color
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            float hue = hsv[0];

            boolean isPurple = (hue > 265 && hue < 285);
            boolean isGreen  = (hue > 95 && hue < 130);

            // Mark slot based on color seen
            if (isPurple) slotType[currSlot] = 1;
            else if (isGreen) slotType[currSlot] = 2;

            // Manual bump → move exactly 1 slot forward
            if (gamepad2.left_bumper) {
                target += SLOT_TICKS;
                sleep(200);
            }

            // Go to purple with pure slot math
            if (gamepad2.dpad_right) {
                int purpleSlot = findSlot(1);
                if (purpleSlot != -1) {
                    int steps = (purpleSlot - currSlot + 3) % 3;
                    target += steps * SLOT_TICKS;
                }
            }

            // Go to green
            if (gamepad2.dpad_left) {
                int greenSlot = findSlot(2);
                if (greenSlot != -1) {
                    int steps = (greenSlot - currSlot + 3) % 3;
                    target += steps * SLOT_TICKS;
                }
            }

            // Manual override
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                double pwr = -gamepad2.left_stick_y;
                spindexer.setPower(pwr);

                // target never decreases
                if (currPos > target) target = currPos;

            } else {
                KineticState ks = new KineticState(currPos, spindexer.getVelocity());
                cs1.setGoal(new KineticState(target));
                spindexer.setPower(cs1.calculate(ks));
            }

            telemetry.addData("Hue", hue);
            telemetry.addData("Current Slot", currSlot);
            telemetry.addData("Slot0", slotType[0]);
            telemetry.addData("Slot1", slotType[1]);
            telemetry.addData("Slot2", slotType[2]);
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }

    private int findSlot(int type) {
        for (int i = 0; i < 3; i++)
            if (slotType[i] == type) return i;
        return -1;
    }

    private int floorMod(int a, int b) {
        int r = a % b;
        return (r < 0) ? r + b : r;
    }
}
