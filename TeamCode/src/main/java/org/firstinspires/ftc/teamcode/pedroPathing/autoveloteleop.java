package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp(name="Auto Velo Tele - RED")
public class autoveloteleop extends LinearOpMode {

    // ===================== DRIVE =====================
    private DcMotor lf, lb, rf, rb;

    // ===================== OTHER SUBSYSTEMS =====================
    private DcMotorEx flywheel, intake, spindexer;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private Servo hood;

    public static NormalizedColorSensor colorSensor;
    DistanceSensor distance;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p = 0.0039, i = 0, d = 0.0000005;
    public static double v = 0.000372, a = 0.7, s = 0.0000005;
    public static double p1 = 0.0009, i1 = 0, d1 = 0;

    float[] hsv = new float[3];

    int ballCount = 0;
    public static int ballshot = 0;
    boolean colorPreviouslyDetected = false;

    // Slot tracking
    int[] ballSlots = new int[]{0, 0, 0}; // 0 empty, 1 purple, 2 green
    boolean sorting = false;
    int[] sortTarget = new int[]{0, 0, 0};

    public static double targetTicksPerSecond = 200;
    public static double shootclose = 1000;
    public static double shootfar = 1600;
    public static double shooteridle = 200;

    // ===================== LIMELIGHT + TURRET FUSED AUTOALIGN =====================
    private Limelight3A limelight;

    private CRServo turretL;
    private CRServo turretR;

    // dedicated encoder you configured in RC as "turret_enc"
    private DcMotorEx turretEnc;

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;

    // ---- your values ----
    public static String pp = "pp";

    public static double deadbandDeg = 2.0;
    public static double edgePauseSec = 0.1;

    // You provided 1.5; we clamp to [0..1] internally
    public static double errAlpha = 1.5;

    public static double forwardPodY = -5.46;
    public static double strafePodX = -1.693;
    public static DistanceUnit ppUnit = DistanceUnit.INCH;

    public static GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static GoBildaPinpointDriver.EncoderDirection forwardDir =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection strafeDir =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static double kP_hold = 0.02;
    public static double kP_track = 0.015;

    public static double maxHoldPower = 0.3;
    public static double maxTrackPower = 0.25;

    public static double maxTurretDeg = 60;
    public static double minTurretDeg = -60;

    public static int pipelineIndex = 2;
    public static double powerSlewPerSec = 1.2;
    public static double searchPower = 0.18;

    // IMPORTANT: applies to BOTH turretL and turretR output
    public static int servoDir = -1;

    public static double ticksPerDeg = 126.42;

    // Save last-known ONLY when TY within ±4°
    public static double lastKnownSaveWindowDeg = 4.0;

    // Stop HOLD<->EDGE chatter around edges
    public static double edgeEnterMarginDeg = 2.0;
    public static double edgeExitMarginDeg = 6.0;

    // Manual turret override behavior
    public static double manualDeadband = 0.08;

    // ===================== LIMELIGHT AIM OFFSET (FIX) =====================
    // + = shift aim left, - = shift aim right
    public static double tyOffsetDeg = -3.5;
    private boolean farmode = false;

    // ===================== TURRET STATE =====================
    private enum TurretMode { TRACK, HOLD, EDGE_SEARCH, IDLE, MANUAL }
    private volatile TurretMode turretMode = TurretMode.IDLE;

    private volatile boolean haveLastKnown = false;
    private volatile double lastKnownAbsDeg = 0.0; // FIELD-ABS direction when tag was centered
    private volatile double tyFilt = 0.0;

    private volatile double lastTurretCmdPower = 0.0;

    private boolean movedoffsetspindexer;

    // EDGE SEARCH sweep state
    private volatile double edgePauseTimer = 0.0;
    private volatile int sweepDir = +1;
    private volatile double sweepTargetDeg = 0.0;

    // For telemetry/debug
    private volatile double robotHeadingDeg = 0.0;
    private volatile double turretRelDeg = 0.0;
    private volatile double tyRaw = 0.0;
    private volatile boolean hasTarget = false;
    private volatile double relUnclampedNeeded = 0.0;
    private volatile double turretRelNeededDeg = 0.0;
    private volatile double holdErrDeg = 0.0;
    private volatile double turretOut = 0.0;
    public static Timer shottimer;
    public static boolean shots=true;

    // ===================== HELPERS =====================
    public double distancefromll(double ta) {
        return (71.7321 * (Math.pow(ta, -0.4550)));
    }
    public double velocityfromdistance(double distance){
        return 562.47005* Math.pow(distance,0.202468);
    }
    public static double hoodpos;

    @Override
    public void runOpMode() {
        shottimer= new Timer();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        rconstants.initHardware(hardwareMap);
        // drive motors
        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;
        boolean shotLatched = false;


        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        // turret
        turretL = rconstants.turretL;
        turretR = rconstants.turretR;

        // limelight
        limelight = rconstants.limelight;
        limelight.pipelineSwitch(pipelineIndex);
        limelight.setPollRateHz(100);
        limelight.start();

        // turret encoder (your new config name)
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pp);
        configurePinpoint(pinpoint);

        // shooter / intake / spindexer
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

        cs1 = ControlSystem.builder()
                .posPid(p1, i1, d1)
                .build();
        cs = ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v, a, s)
                .build();

        int target = 0;

        // ===================== DRIVE THREAD (KEEP YOUR LOGIC) =====================
        Thread driveThread = new Thread(() -> {
            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                lf.setPower((y + x + rx) / denom);
                lb.setPower((y - x + rx) / denom);
                rf.setPower((y - x - rx) / denom);
                rb.setPower((y + x - rx) / denom);

                if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper) {
                    transfermover.setPosition(rconstants.transfermoverscore);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if (gamepad2.right_trigger > 0 && gamepad2.right_bumper) {
                    transfermover.setPosition(rconstants.transfermoverfull);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if (gamepad2.right_trigger == 0) {
                    transfermover.setPosition(rconstants.transfermoveridle);
                    transfer.setPower(0);
                }

                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
        });

        // ===================== TURRET THREAD (ALWAYS RUNS) =====================
        Thread turretThread = new Thread(() -> {

            long lastNanos = System.nanoTime();
            // start in AUTO (not manual)
            turretMode = TurretMode.IDLE;

            while (opModeIsActive()) {

                long now = System.nanoTime();
                double dt = (now - lastNanos) / 1e9;
                lastNanos = now;
                if (dt <= 0) dt = 0.02;

                // Always update pinpoint + heading
                pinpoint.update();
                Pose2D pose = pinpoint.getPosition();
                robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

                // Turret relative angle from encoder
                turretRelDeg = turretEnc.getCurrentPosition() / ticksPerDeg;

                // Limelight read
                LLResult res = limelight.getLatestResult();
                hasTarget = (res != null) && res.isValid();

                // ===================== FIX: APPLY TY OFFSET HERE =====================
                tyRaw = hasTarget ? (res.getTy() + tyOffsetDeg) : 0.0;

                // Filter TY
                double alpha = clamp(errAlpha, 0.0, 1.0);
                tyFilt = (1.0 - alpha) * tyFilt + alpha * tyRaw;

                // Manual override only while stick is moved; release -> go back to auto
                double manualStick = -0.7*gamepad2.right_stick_x;
                boolean manualNow = Math.abs(manualStick) > manualDeadband;

                // Update lastKnownAbs direction when tag is well-centered
                if (hasTarget && Math.abs(tyFilt) <= lastKnownSaveWindowDeg) {
                    lastKnownAbsDeg = angleWrapDeg(robotHeadingDeg + turretRelDeg);
                    haveLastKnown = true;
                }

                // If manual: drive turret directly, but ALSO keep lastKnownAbs tracking turret direction
                if (manualNow) {
                    turretMode = TurretMode.MANUAL;

                    lastKnownAbsDeg = angleWrapDeg(robotHeadingDeg + turretRelDeg);
                    haveLastKnown = true;

                    double cmd = clamp(manualStick, -1.0, +1.0);

                    // soft limits in manual too
                    if (turretRelDeg <= minTurretDeg && cmd < 0) cmd = 0;
                    if (turretRelDeg >= maxTurretDeg && cmd > 0) cmd = 0;

                    setTurretPower(servoDir * cmd);
                    lastTurretCmdPower = cmd;

                    // reset edge-search state so it doesn't resume mid-sweep
                    edgePauseTimer = 0.0;
                    sweepTargetDeg = 0.0;

                    continue;
                }

                // Not manual => AUTO
                relUnclampedNeeded = 0.0;
                turretRelNeededDeg = 0.0;
                holdErrDeg = 0.0;

                if (haveLastKnown) {
                    relUnclampedNeeded = angleWrapDeg(lastKnownAbsDeg - robotHeadingDeg);
                    turretRelNeededDeg = clamp(relUnclampedNeeded, minTurretDeg, maxTurretDeg);
                    holdErrDeg = angleWrapDeg(turretRelNeededDeg - turretRelDeg);
                }

                // Mode selection (with hysteresis)
                TurretMode next;
                if (hasTarget) {
                    next = TurretMode.TRACK;
                } else if (!haveLastKnown) {
                    next = TurretMode.IDLE;
                } else {
                    boolean outsideEnter =
                            (relUnclampedNeeded > (maxTurretDeg + edgeEnterMarginDeg)) ||
                                    (relUnclampedNeeded < (minTurretDeg - edgeEnterMarginDeg));

                    boolean insideExit =
                            (relUnclampedNeeded < (maxTurretDeg - edgeExitMarginDeg)) &&
                                    (relUnclampedNeeded > (minTurretDeg + edgeExitMarginDeg));

                    if (turretMode == TurretMode.EDGE_SEARCH) {
                        next = insideExit ? TurretMode.IDLE : TurretMode.EDGE_SEARCH;
                    } else {
                        next = outsideEnter ? TurretMode.EDGE_SEARCH : TurretMode.IDLE;
                    }

                }
                turretMode = next;

                // Command power
                double cmdPower;

                if (turretMode == TurretMode.TRACK) {
                    // drive TY to 0
                    if (Math.abs(tyFilt) <= deadbandDeg) {
                        cmdPower = 0.0;
                    } else {
                        cmdPower = -kP_track * tyFilt;
                        cmdPower = clamp(cmdPower, -maxTrackPower, +maxTrackPower);
                    }
               /* } else if (turretMode == TurretMode.HOLD) {
                    cmdPower = kP_hold * holdErrDeg;
                    cmdPower = clamp(cmdPower, -maxHoldPower, +maxHoldPower);*/
                } else {
                    cmdPower = 0.0;
                    edgePauseTimer = 0.0;
                    sweepTargetDeg = Double.NaN;
                }
                /*else if (turretMode == TurretMode.EDGE_SEARCH) {

                    double desiredEdge = (relUnclampedNeeded >= 0) ? maxTurretDeg : minTurretDeg;

                    if (Double.isNaN(sweepTargetDeg)) {
                        sweepTargetDeg = desiredEdge;
                        sweepDir = (sweepTargetDeg > turretRelDeg) ? +1 : -1;
                        edgePauseTimer = 0.0;
                    }

                    boolean atEdge = Math.abs(turretRelDeg - sweepTargetDeg) <= 2.0;

                    if (atEdge) {
                        edgePauseTimer += dt;

                        if (edgePauseTimer >= edgePauseSec) {
                            sweepTargetDeg = (sweepTargetDeg > 0) ? minTurretDeg : maxTurretDeg;
                            sweepDir = (sweepTargetDeg > turretRelDeg) ? +1 : -1;
                            edgePauseTimer = 0.0;
                        }
                        cmdPower = 0.0;
                    } else {
                        edgePauseTimer = 0.0;
                        cmdPower = sweepDir * searchPower;
                    }*/



                // Soft limits
                if (turretRelDeg <= minTurretDeg && cmdPower < 0) cmdPower = 0.0;
                if (turretRelDeg >= maxTurretDeg && cmdPower > 0) cmdPower = 0.0;

                // Slew limit (smooth)
                double maxDelta = powerSlewPerSec * dt;
                cmdPower = clamp(cmdPower, lastTurretCmdPower - maxDelta, lastTurretCmdPower + maxDelta);
                lastTurretCmdPower = cmdPower;

                turretOut = servoDir * cmdPower;
                setTurretPower(turretOut);
            }
        });

        // IMPORTANT: init edge-search sentinel
        sweepTargetDeg = Double.NaN;

        waitForStart();

        driveThread.start();
        turretThread.start();

        // ============================================================
        //                     MAIN LOOP
        // ============================================================
        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();
            if(llResult.isValid()){
                if(distancefromll(llResult.getTa())>110){
                    farmode=true;
                } else{
                    farmode=false;
                }
                if(distancefromll(llResult.getTa())>=60){
                    targetTicksPerSecond = velocityfromdistance(distancefromll(llResult.getTa()));
                    hood.setPosition(rconstants.hoodtop);
                } else{
                    if (gamepad2.y) {
                        targetTicksPerSecond = rconstants.shootfar;
                        hood.setPosition(rconstants.hoodtop);
                        farmode=true;
                    }
                    else if (gamepad2.b){
                        targetTicksPerSecond = rconstants.shootclose;
                        hood.setPosition(rconstants.hoodtop);
                        farmode=false;
                    }
                    else if (gamepad2.a) {
                        targetTicksPerSecond = rconstants.shooteridle;
                        hood.setPosition(rconstants.hoodbottom);
                        farmode=false;
                    } else{
                        hood.setPosition(rconstants.hoodbottom);
                        targetTicksPerSecond=rconstants.shooteridle;
                        farmode=false;
                    }
                }
            }


            if(gamepad2.ps){
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                target=0;
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            telemetry.addData("spindexer_pos", spindexer.getCurrentPosition());
           if(llResult.isValid()){
                telemetry.addData("TA: ", llResult.getTa());
            }
            telemetry.addData("distance", distancefromll(llResult.getTa()));
            telemetry.addData("distance of spindexer", distance.getDistance(DistanceUnit.CM));
            telemetry.update();


        }
    }

    // ===================== Pinpoint setup =====================
    private void configurePinpoint(GoBildaPinpointDriver ppDevice) {
        ppDevice.setOffsets(forwardPodY, strafePodX, ppUnit);
        ppDevice.setEncoderResolution(podType);
        ppDevice.setEncoderDirections(forwardDir, strafeDir);
        ppDevice.resetPosAndIMU();
    }

    // ===================== Turret power helper =====================
    private void setTurretPower(double pwr) {
        turretL.setPower(pwr);
        turretR.setPower(pwr);
    }

    // Wrap degrees to [-180, +180]
    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
