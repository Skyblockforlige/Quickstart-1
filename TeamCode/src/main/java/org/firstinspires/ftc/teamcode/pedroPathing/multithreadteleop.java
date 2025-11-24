package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp
public class multithreadteleop extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
    private DcMotorEx flywheel, intake, spindexer;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;

    public static NormalizedColorSensor colorSensor;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;
    public static double p1=0.0009,i1=0,d1=0;

    float[] hsv = new float[3];

    int ballCount = 0;
    boolean colorPreviouslyDetected = false;

    // Slot tracking
    int[] ballSlots = new int[]{0,0,0}; // 0 empty, 1 purple, 2 green
    boolean sorting = false;
    int[] sortTarget = new int[]{0,0,0};

    public static double targetTicksPerSecond=200;
    public static double shootclose = 1250;
    public static double shootfar=1600;
    public static double shooteridle = 200;

    @Override
    public void runOpMode(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rconstants.initHardware(hardwareMap);

        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel = rconstants.flywheel;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = rconstants.intake;

        spindexer = rconstants.spindexer;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = rconstants.transfer;
        transfermover = rconstants.transfermover;
        transfermover.setPosition(rconstants.transfermoveridle);

        colorSensor = rconstants.colorSensor;
        colorSensor.setGain(2.7f);

        cs1 = ControlSystem.builder().posPid(p1,i1,d1).build();
        cs = ControlSystem.builder().velPid(p,i,d).basicFF(v,a,s).build();

        int target = 0;

        // ---------------------- DRIVE THREAD ----------------------
        Thread g1 = new Thread(() -> {
            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
                lf.setPower((y+x+rx)/denom);
                lb.setPower((y-x+rx)/denom);
                rf.setPower((y-x-rx)/denom);
                rb.setPower((y+x-rx)/denom);

                if(gamepad2.right_trigger>0 && !gamepad2.right_bumper){
                    transfermover.setPosition(rconstants.transfermoverscore);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if(gamepad2.right_trigger>0 && gamepad2.right_bumper){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if(gamepad2.right_trigger==0){
                    transfermover.setPosition(rconstants.transfermoveridle);
                    transfer.setPower(0);
                }
            }
        });

        waitForStart();
        g1.start();

        // ============================================================
        //                     MAIN LOOP
        // ============================================================
        while (opModeIsActive()) {

            // ---------- INTAKE ----------
            intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.05;

            // ---------- READ COLOR ----------
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            float hue = hsv[0];
            boolean isPurple = (hue > 200 && hue < 300);
            boolean isGreen  = (hue > 95  && hue < 200);

            boolean colorDetected = (isPurple || isGreen);

            // ---------- BALL DETECTION ----------
            if (intakeRunning && colorDetected && !colorPreviouslyDetected && ballCount < 3) {
                sleep(300);
                target += rconstants.movespindexer;

                if (isPurple) ballSlots[ballCount] = 1;
                if (isGreen)  ballSlots[ballCount] = 2;

                ballCount++;
                colorPreviouslyDetected = true;
            }

            if (!colorDetected) {
                colorPreviouslyDetected = false;
            }

            // ---------- SHOOTER ----------
            if (gamepad2.y) targetTicksPerSecond = rconstants.shootfar;
            if (gamepad2.b) targetTicksPerSecond = rconstants.shootclose;
            if (gamepad2.a) targetTicksPerSecond = rconstants.shooteridle;

            // Reset only ballCount (not slots)
            if (gamepad2.x) {
                ballCount = 0;
            }

            // ---------- SORT BUTTONS ----------
            if (gamepad2.dpad_up && ballCount == 3 && !sorting) {
                //PPG
                sortTarget = new int[]{1,2,1};
                sorting = true;
            }
            if (gamepad2.dpad_left && ballCount == 3 && !sorting) {
                //PGP
                sortTarget = new int[]{2,1,1};
                sorting = true;
            }
            if (gamepad2.dpad_right && ballCount == 3 && !sorting) {
                //GPP
                sortTarget = new int[]{1,1,2};
                sorting = true;
            }

            // ---------- FORWARD-ONLY SORTING ----------
            if (sorting) {

                if (!(ballSlots[0] == sortTarget[0] &&ballSlots[1]==sortTarget[1]&& ballSlots[2]==sortTarget[2])) {

                    target += rconstants.movespindexer;

                    int temp = ballSlots[0];
                    ballSlots[0] = ballSlots[1];
                    ballSlots[1] = ballSlots[2];
                    ballSlots[2] = temp;

                    sleep(250);

                } else {
                    sorting = false;
                }
            }

            // ---------- SPINDEXER PID ----------
            KineticState current2 = new KineticState(spindexer.getCurrentPosition(), spindexer.getVelocity());
            cs1.setGoal(new KineticState(target));

            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                spindexer.setPower(cs1.calculate(current2));
            } else {
                spindexer.setPower(0.7*-gamepad2.left_stick_y);
                target = spindexer.getCurrentPosition();
            }

            // ---------- SHOOTER ----------
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            KineticState current1 = new KineticState(flywheel.getCurrentPosition(),flywheel.getVelocity());
            flywheel.setPower(cs.calculate(current1));
            if(gamepad2.left_bumper){
                int moveamount = rconstants.movespindexer-(target%rconstants.movespindexer);
                target+=moveamount;
                sleep(300);
            }

            // ---------- TELEMETRY ----------
                telemetry.addData("Hue", hue);
                telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Slots", ballSlots[0] + "," + ballSlots[1] + "," + ballSlots[2]);
            telemetry.addData("Sorting", sorting);
            telemetry.addData("Sort Target", sortTarget[0] + "," + sortTarget[1] + "," + sortTarget[2]);
            telemetry.addData("Target", target);
            telemetry.update();
        }
    }
}
