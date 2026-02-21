package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Turret LookAtTag + Pinpoint fallback (NO manual turret control)
 *
 * - TRACK: if Limelight has target -> uses TY (because camera is vertical mounted) to center tag
 * - HOLD: if no target but we have last-known abs tag direction -> hold turret to look there using CURRENT robot heading
 * - EDGE_SEARCH: only when the needed turret angle to look at last-known abs direction is outside [-90,+90]
 *      -> drives turret to an edge and sweeps until reacquire
 *
 * ALSO: robot rotation only (no translation) using gamepad1 right_stick_x (rx),
 * matching the same wheel names: lf, lb, rf, rb.
 *
 * IMPORTANT NOTE:
 * If your "lf" motor is BOTH a drive motor AND your turret encoder source, then rotating the robot
 * will change lf encoder ticks and will corrupt turret angle. This opmode assumes lf encoder truly
 * represents turret angle.
 */
@Configurable
@Config
@Disabled
@TeleOp(name = "TurretFused (TY + Pinpoint Hold + EdgeSearch)", group = "TeleOp")
public class TurretLookAtTag_NoPID_NoSearch extends LinearOpMode {

    // ================== DRIVE (rotation-only) ==================
    private DcMotor lf, lb, rf, rb;

    // ================== TURRET ==================
    private DcMotorEx turretEnc;   // encoder source (you said: lf encoder in your setup)
    private CRServo turretL;       // your turret CRServo (single)

    // ================== VISION ==================
    private Limelight3A limelight;

    // ================== ODOM / HEADING ==================
    private GoBildaPinpointDriver pinpoint;

    // ================== PROVIDED TUNABLES (YOUR VALUES) ==================
    public static String pp = "pp";

    public static double deadbandDeg = 2.0;
    public static double edgePauseSec = 0.1;

    // NOTE: you listed errAlpha=1.5 (that is invalid for a normal low-pass filter).
    // We clamp it internally to [0..1] so it won't explode.
    public static double errAlpha = 1.5;

    public static double forwardPodY = -5.46;
    public static double strafePodX  = -1.693;
    public static DistanceUnit ppUnit = DistanceUnit.INCH;

    public static GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static GoBildaPinpointDriver.EncoderDirection forwardDir =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection strafeDir =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static double kP_hold = 0.02;
    public static double kP_track = 0.015;

    public static double maxHoldPower  = 0.3;
    public static double maxTrackPower = 0.25;

    public static double maxTurretDeg = 90.0;
    public static double minTurretDeg = -90.0;

    public static int pipelineIndex = 2;

    public static double powerSlewPerSec = 1.2;
    public static double searchPower = 0.18;

    public static int servoDir = -1;

    public static double ticksPerDeg = 126.42;

    // ================== EXTRA BEHAVIOR SETTINGS ==================
    // Save last-known ONLY when we were close to centered (your request: TY within ±4deg)
    public static double lastKnownSaveWindowDeg = 4.0;

    // Prevent HOLD <-> EDGE_SEARCH chattering around edges (your 86/92 issue)
    public static double edgeEnterMarginDeg = 2.0;   // enter edge mode if needed beyond limit by this much
    public static double edgeExitMarginDeg  = 6.0;   // exit edge mode only when needed back within limit by this much

    // ================== STATE ==================
    private enum Mode { TRACK, HOLD, EDGE_SEARCH, IDLE }
    private Mode mode = Mode.IDLE;

    private boolean haveLastKnown = false;
    private double lastKnownAbsDeg = 0.0;      // FIELD-ABS direction where tag was last centered
    private double tyFilt = 0.0;

    private double lastPower = 0.0;
    private long lastNanos = 0;

    // EDGE SEARCH sweep state
    private double edgePauseTimer = 0.0;
    private int sweepDir = +1;                // +1 -> toward +90, -1 -> toward -90
    private double sweepTargetDeg = 0.0;      // current sweep goal edge

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        // ===== Wheel names from your teleop =====
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");

        // ===== Turret (encoder source is lf per your setup) =====
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");

        turretL   = hardwareMap.crservo.get("turretL");

        // ===== Limelight =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineIndex);
        limelight.setPollRateHz(100);
        limelight.start();

        // ===== Pinpoint =====
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pp);
        configurePinpoint(pinpoint);

        // Encoder init
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastNanos = System.nanoTime();

        waitForStart();

        while (opModeIsActive()) {

            // ================== dt ==================
            long now = System.nanoTime();
            double dt = (now - lastNanos) / 1e9;
            lastNanos = now;
            if (dt <= 0) dt = 0.02;

            // ================== ROTATE ROBOT ONLY (gamepad1) ==================
            double rotate_rx = gamepad1.right_stick_x;
            // rotation only
            lf.setPower( rotate_rx);
            lb.setPower( rotate_rx);
            rf.setPower(-rotate_rx);
            rb.setPower(-rotate_rx);

            // ================== Heading from Pinpoint ==================
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            // ================== Turret relative angle from encoder ==================
            double turretRelDeg = turretEnc.getCurrentPosition() / ticksPerDeg;

            // ================== Limelight TY ==================
            LLResult res = limelight.getLatestResult();
            boolean hasTarget = (res != null) && res.isValid();
            double tyRaw = hasTarget ? res.getTy() : 0.0;

            // Filter ty
            double a = clamp(errAlpha, 0.0, 1.0);
            tyFilt = (1.0 - a) * tyFilt + a * tyRaw;

            // ================== Save last-known ABS direction when centered-ish ==================
            if (hasTarget && Math.abs(tyFilt) <= lastKnownSaveWindowDeg) {
                // When centered, the camera is pointing at tag. Record FIELD-ABS direction:
                // abs = robotHeading + turretRel
                lastKnownAbsDeg = angleWrapDeg(robotHeadingDeg + turretRelDeg);
                haveLastKnown = true;
            }

            // ================== Compute needed angle to look at lastKnown using CURRENT heading ==================
            double relUnclampedNeeded = 0.0;
            if (haveLastKnown) {
                relUnclampedNeeded = angleWrapDeg(lastKnownAbsDeg - robotHeadingDeg);
            }

            // For debugging “which way should it go”
            double turretRelNeededDeg = 0.0;
            double holdErrDeg = 0.0;
            if (haveLastKnown) {
                turretRelNeededDeg = clamp(relUnclampedNeeded, minTurretDeg, maxTurretDeg);
                holdErrDeg = angleWrapDeg(turretRelNeededDeg - turretRelDeg);
            }

            // ================== MODE SELECTION (with hysteresis to stop HOLD/EDGE flapping) ==================
            if (hasTarget) {
                mode = Mode.TRACK;
            } else if (!haveLastKnown) {
                mode = Mode.IDLE;
            } else {
                boolean outsideEnter =
                        (relUnclampedNeeded > (maxTurretDeg + edgeEnterMarginDeg)) ||
                                (relUnclampedNeeded < (minTurretDeg - edgeEnterMarginDeg));

                boolean insideExit =
                        (relUnclampedNeeded < (maxTurretDeg - edgeExitMarginDeg)) &&
                                (relUnclampedNeeded > (minTurretDeg + edgeExitMarginDeg));

                if (mode == Mode.EDGE_SEARCH) {
                    // stay in EDGE until clearly back inside
                    mode = insideExit ? Mode.HOLD : Mode.EDGE_SEARCH;
                } else {
                    mode = outsideEnter ? Mode.EDGE_SEARCH : Mode.HOLD;
                }
            }

            // ================== COMMAND POWER (before servoDir) ==================
            double cmdPower = 0.0;

            if (mode == Mode.TRACK) {
                // drive TY to 0
                if (Math.abs(tyFilt) <= deadbandDeg) {
                    cmdPower = 0.0;
                } else {
                    cmdPower = -kP_track * tyFilt;
                    cmdPower = clamp(cmdPower, -maxTrackPower, +maxTrackPower);
                }

            } else if (mode == Mode.HOLD) {
                // Hold turret pointed at lastKnownAbs using CURRENT heading
                // holdErrDeg already = needed - current (wrapped)
                cmdPower = kP_hold * holdErrDeg;
                cmdPower = clamp(cmdPower, -maxHoldPower, +maxHoldPower);

            } else if (mode == Mode.EDGE_SEARCH) {
                // Only used when the needed angle is outside limits.
                // Strategy:
                // 1) choose nearer “correct” edge based on sign of needed
                // 2) go to that edge, pause briefly, then sweep across to the other edge and bounce
                double desiredEdge = (relUnclampedNeeded >= 0) ? maxTurretDeg : minTurretDeg;

                // Initialize sweep target/direction when we first enter EDGE_SEARCH
                if (edgePauseTimer == 0.0 && sweepTargetDeg == 0.0) {
                    sweepTargetDeg = desiredEdge;
                    sweepDir = (sweepTargetDeg > turretRelDeg) ? +1 : -1;
                }

                // Are we at the current sweep target edge?
                boolean atEdge = Math.abs(turretRelDeg - sweepTargetDeg) <= 2.0;

                if (atEdge) {
                    edgePauseTimer += dt;

                    if (edgePauseTimer >= edgePauseSec) {
                        // flip sweep target
                        sweepTargetDeg = (sweepTargetDeg > 0) ? minTurretDeg : maxTurretDeg;
                        sweepDir = (sweepTargetDeg > turretRelDeg) ? +1 : -1;
                        edgePauseTimer = 0.0;
                    }

                    cmdPower = 0.0; // pause at edge
                } else {
                    edgePauseTimer = 0.0;
                    cmdPower = sweepDir * searchPower;
                }

            } else {
                // IDLE
                cmdPower = 0.0;
                edgePauseTimer = 0.0;
                sweepTargetDeg = 0.0; // reset
            }

            // ================== SOFT LIMITS (don’t push beyond min/max) ==================
            if (turretRelDeg <= minTurretDeg && cmdPower < 0) cmdPower = 0.0;
            if (turretRelDeg >= maxTurretDeg && cmdPower > 0) cmdPower = 0.0;

            // ================== SLEW LIMIT ==================
            double maxDelta = powerSlewPerSec * dt;
            cmdPower = clamp(cmdPower, lastPower - maxDelta, lastPower + maxDelta);
            lastPower = cmdPower;

            // ================== OUTPUT TO SERVO ==================
            double turretOut = servoDir * cmdPower;
            turretL.setPower(turretOut);

            // ================== TELEMETRY ==================
            telemetry.addData("MODE", mode);
            telemetry.addData("hasTarget", hasTarget);
            telemetry.addData("tyRaw(deg)", tyRaw);
            telemetry.addData("tyFilt(deg)", tyFilt);

            telemetry.addData("robotHeadingDeg", robotHeadingDeg);
            telemetry.addData("turretRelDeg", turretRelDeg);

            telemetry.addData("haveLastKnown", haveLastKnown);
            telemetry.addData("lastKnownAbsDeg", lastKnownAbsDeg);

            telemetry.addData("relUnclampedNeeded", haveLastKnown ? relUnclampedNeeded : 0.0);

            // NEW: show where turret needs to be + error sign
            telemetry.addData("turretRelNeededDeg", haveLastKnown ? turretRelNeededDeg : 0.0);
            telemetry.addData("holdErrDeg (needed-current)", haveLastKnown ? holdErrDeg : 0.0);

            telemetry.addData("cmdPower(beforeDir)", cmdPower);
            telemetry.addData("servoDir", servoDir);
            telemetry.addData("turretOut", turretOut);

            telemetry.addData("edgePauseTimer", edgePauseTimer);
            telemetry.addData("sweepTargetDeg", sweepTargetDeg);
            telemetry.addData("sweepDir", sweepDir);

            telemetry.addData("rotate_rx", rotate_rx);
            telemetry.update();
        }
    }

    private void configurePinpoint(GoBildaPinpointDriver ppDevice) {
        ppDevice.setOffsets(forwardPodY, strafePodX, ppUnit);
        ppDevice.setEncoderResolution(podType);
        ppDevice.setEncoderDirections(forwardDir, strafeDir);
        ppDevice.resetPosAndIMU();
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
