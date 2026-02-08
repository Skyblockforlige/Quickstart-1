package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp(name = "TELE W TURRET FINAL RED", group = "0")
public class teleFINALwturret extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor lf, lb, rf, rb;

    // ================= SUBSYSTEMS =================
    private DcMotorEx flywheel, intake, spindexer;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private Servo hood;

    // ================= TURRET =================
    private CRServo turretL, turretR;
    private DcMotorEx turretEnc;
    private Limelight3A limelight;

    private enum TurretMode { TRACK, HOLD, MANUAL, IDLE }
    private volatile TurretMode turretMode = TurretMode.IDLE;

    public static double ticksPerDeg = 126.42;
    public static double minTurretDeg = -80;
    public static double maxTurretDeg = 80;

    public static double kP_track = 0.015;
    public static double kP_hold  = 0.02;

    public static double maxTrackPower = 0.25;
    public static double maxHoldPower  = 0.3;

    public static double deadbandDeg = 2.0;
    public static double tyOffsetDeg = -3.5;
    public static double manualDeadband = 0.08;
    public static int servoDir = -1;

    private volatile boolean haveLastKnown = false;
    private volatile double lastKnownTurretDeg = 0;

    // ================= OTHER =================
    public static NormalizedColorSensor colorSensor;
    DistanceSensor distance;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;
    public static double p1=0.0009,i1=0,d1=0;

    public static double targetTicksPerSecond=200;
    public static double shootclose = 1000;
    public static double shootfar=1600;
    public static double shooteridle = 200;

    public static boolean autoalign = false;
    public static boolean farmode = false;

    float[] hsv = new float[3];
    int ballCount = 0;
    boolean colorPreviouslyDetected = false;
    int[] ballSlots = new int[]{0,0,0};
    boolean sorting = false;
    int[] sortTarget = new int[]{0,0,0};
    private boolean movedoffsetspindexer;

    public double distancefromll(double ta) {
        return 71.7321 * Math.pow(ta, -0.4550);
    }

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        rconstants.initHardware(hardwareMap);

        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        turretL = rconstants.turretL;
        turretR = rconstants.turretR;

        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = rconstants.limelight;
        limelight.pipelineSwitch(1);
        limelight.start();

        flywheel = rconstants.flywheel;
        hood = rconstants.hood;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = rconstants.intake;
        spindexer = rconstants.spindexer;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = rconstants.transfer;
        transfermover = rconstants.transfermover;
        transfermover.setPosition(rconstants.transfermoveridle);

        colorSensor = rconstants.colorSensor;
        colorSensor.setGain(rconstants.csgain);
        distance = (DistanceSensor) colorSensor;

        cs1 = ControlSystem.builder().posPid(p1,i1,d1).build();
        cs = ControlSystem.builder().velPid(p,i,d).basicFF(v,a,s).build();

        int target = 0;

        // ================= DRIVE THREAD =================
        Thread driveThread = new Thread(() -> {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
                lf.setPower((y+x+rx)/denom);
                lb.setPower((y-x+rx)/denom);
                rf.setPower((y-x-rx)/denom);
                rb.setPower((y+x-rx)/denom);

                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
        });

        // ================= TURRET THREAD =================
        Thread turretThread = new Thread(() -> {
            while (opModeIsActive()) {

                LLResult res = limelight.getLatestResult();
                boolean hasTarget = res != null && res.isValid();

                double turretDeg = turretEnc.getCurrentPosition() / ticksPerDeg;
                double manual = gamepad2.right_stick_x;

                // MANUAL
                if (Math.abs(manual) > manualDeadband) {
                    turretMode = TurretMode.MANUAL;

                    double cmd = clamp(manual, -1, 1);
                    if (turretDeg <= minTurretDeg && cmd < 0) cmd = 0;
                    if (turretDeg >= maxTurretDeg && cmd > 0) cmd = 0;

                    turretL.setPower(servoDir * cmd);
                    turretR.setPower(servoDir * cmd);

                    lastKnownTurretDeg = turretDeg;
                    haveLastKnown = true;
                }

                // TRACK
                else if (autoalign && hasTarget) {
                    turretMode = TurretMode.TRACK;

                    double ty = res.getTy() + tyOffsetDeg;

                    if (Math.abs(ty) <= deadbandDeg) {
                        turretL.setPower(0);
                        turretR.setPower(0);
                    } else {
                        double cmd = clamp(-kP_track * ty,
                                -maxTrackPower, maxTrackPower);
                        turretL.setPower(servoDir * cmd);
                        turretR.setPower(servoDir * cmd);
                    }

                    lastKnownTurretDeg = turretDeg;
                    haveLastKnown = true;
                }

                // HOLD
                else if (autoalign && haveLastKnown) {
                    turretMode = TurretMode.HOLD;

                    double err = lastKnownTurretDeg - turretDeg;
                    double cmd = clamp(kP_hold * err,
                            -maxHoldPower, maxHoldPower);

                    turretL.setPower(servoDir * cmd);
                    turretR.setPower(servoDir * cmd);
                }

                // IDLE
                else {
                    turretMode = TurretMode.IDLE;
                    turretL.setPower(0);
                    turretR.setPower(0);
                }
            }
        });

        waitForStart();
        driveThread.start();
        turretThread.start();

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            LLResult llResult = limelight.getLatestResult();



            if(gamepad2.ps){
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                target=0;
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad2.back){
                autoalign=!autoalign;
                sleep(300);
            }
            if(ballCount==3&&!movedoffsetspindexer){
                sleep(100);
                target+=movespindexer/2;
                target-=750;
                movedoffsetspindexer=true;
            }
            if(gamepad2.left_trigger >0.1){
                target+=movespindexer/2;
            }
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.05;

            // ---------- READ COLOR ----------
            if(spindexer.getCurrentPosition()%rconstants.movespindexer <= 100 || spindexer.getCurrentPosition()%rconstants.movespindexer >= rconstants.movespindexer-100) {
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
            }

            float hue = hsv[0];

            boolean isPurple = (hue > 195 && hue < 220);
            boolean isGreen = (hue > 150 && hue < 170);

            boolean colorDetected = (isPurple || isGreen);

            // ---------- BALL DETECTION ----------
            if (intakeRunning && colorDetected && !colorPreviouslyDetected && ballCount < 3) {
                if(distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<7) {
                    //sleep(200);
                    target += rconstants.movespindexer;

                    if (isPurple) ballSlots[ballCount] = 1;
                    if (isGreen) ballSlots[ballCount] = 2;

                    ballCount++;
                    colorPreviouslyDetected = true;
                }
            }

            if (!colorDetected) {
                colorPreviouslyDetected = false;
            }

            // ---------- SHOOTER ----------
            if (gamepad2.y) {
                targetTicksPerSecond = rconstants.shootfar;
                hood.setPosition(rconstants.hoodtop);
                farmode=true;
            }
            if (gamepad2.b){
                targetTicksPerSecond = rconstants.shootclose;
                hood.setPosition(rconstants.hoodtop);
                farmode=false;
            }
            if (gamepad2.a) {
                targetTicksPerSecond = rconstants.shooteridle;
                hood.setPosition(rconstants.hoodbottom);
                farmode=false;
            }

            // Reset only ballCount (not slots)
            if (gamepad2.x) {
                ballCount = 0;
                movedoffsetspindexer=false;
            }

            // ---------- SORT BUTTONS ----------
            if (gamepad2.dpad_right && ballCount == 3 && !sorting) {
                //PPG
                sortTarget = new int[]{1,2,1};
                sorting = true;
            }
            if (gamepad2.dpad_up && ballCount == 3 && !sorting) {
                //PGP
                sortTarget = new int[]{2,1,1};
                sorting = true;
            }
            if (gamepad2.dpad_left && ballCount == 3 && !sorting) {
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
                spindexer.setPower(-cs1.calculate(current2));
            } else if(farmode){
                spindexer.setPower(0.6*gamepad2.left_stick_y);
                target = spindexer.getCurrentPosition();
            } else{
                spindexer.setPower(gamepad2.left_stick_y);
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

            while(gamepad1.left_trigger>0){
                if(ballCount==3&&!isGreen){
                    target+=rconstants.movespindexer;
                    sleep(300);
                }
            }



            // ---------- TELEMETRY ----------
            telemetry.addData("Hue", hue);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Slots", ballSlots[0] + "," + ballSlots[1] + "," + ballSlots[2]);
            telemetry.addData("Sorting", sorting);
            telemetry.addData("Sort Target", sortTarget[0] + "," + sortTarget[1] + "," + sortTarget[2]);
            telemetry.addData("Target", target);
            telemetry.addData("Autoalign:", autoalign);
            telemetry.addData("spindexer_pos", spindexer.getCurrentPosition());
            telemetry.addData("distance", distancefromll(llResult.getTa()));
            telemetry.addData("distance of spindexer", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
