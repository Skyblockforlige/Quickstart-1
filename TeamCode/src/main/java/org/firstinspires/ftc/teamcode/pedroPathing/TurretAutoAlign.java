package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp(name = "Turret AutoAlign (NO SEARCH, stable lock)", group = "TeleOp")
public class TurretAutoAlign extends LinearOpMode {

    DcMotorEx lf;          // encoder source motor (your turret encoder)
    CRServo turretL;       // continuous rotation servo

    GoBildaPinpointDriver pinpoint;
    Limelight3A limelight;

    ControlSystem cs1;

    // ===== PID =====
    public static double p = 0.00035, i = 0.0000000005, d = 0.0000000002;

    // ===== Mechanics =====
    public static double ticksperdegree = 126.42;
    public static double turretZeroOffsetDeg = 0.0;
    public static int turretDir = 1; // left +, right -

    // ===== Limits =====
    public static double minTurretDeg = -90.0;
    public static double maxTurretDeg =  90.0;

    // ===== Limelight =====
    public static double txDeadbandDeg = 2.5;
    public static double camYawOffsetDeg = 0.0;   // tune if LL is mounted offset
    public static double visionGain = 1.0;

    // Filter + stability gate (kills flicker twitch)
    public static double txAlpha = 0.10;          // 0.05 smoother, 0.2 faster
    public static int validFramesToUpdate = 3;

    // Smooth lock (prevents step-jerk)
    public static double lockBlend = 0.18;        // 0.10 smoother, 0.30 faster

    // Servo chatter killers
    public static int holdZoneTicks = 20;
    public static double maxServoPower = 0.70;

    // ===== Pinpoint config =====
    public static String PINPOINT_NAME = "pp";
    public static double forwardPodY = -5.46;
    public static double strafePodX  = -1.693;
    public static DistanceUnit ppUnit = DistanceUnit.INCH;
    public static GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static GoBildaPinpointDriver.EncoderDirection forwardDir =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection strafeDir =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // ===== State =====
    public static int targetTicks = 0;

    private double turretAbsTargetDeg = 0.0; // field-locked direction
    private boolean absInit = false;

    private double txFilt = 0.0;
    private int validCount = 0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        turretL = hardwareMap.crservo.get("turretL");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        configurePinpoint(pinpoint);

        limelight.pipelineSwitch(2);
        limelight.setPollRateHz(100);
        limelight.start();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cs1 = ControlSystem.builder()
                .posPid(p, i, d)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            // ===== Heading =====
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            // ===== Limelight =====
            LLResult res = limelight.getLatestResult();
            boolean hasTarget = (res != null) && res.isValid();
            double txRaw = hasTarget ? res.getTx() : 0.0;
            double txCorr = txRaw + camYawOffsetDeg;

            // Low pass filter tx (even if target flickers)
            txFilt = (1.0 - txAlpha) * txFilt + txAlpha * txCorr;

            // ===== Manual override =====
            boolean manual = Math.abs(gamepad2.left_stick_x) > 0.05;

            // ===== Current turret relative angle =====
            double turretRelNowDeg = ticksToDeg(lf.getCurrentPosition());

            // ===== Init lock: you said you will start already looking at the tag =====
            if (!absInit) {
                // latch current direction as field lock
                turretAbsTargetDeg = angleWrapDeg(robotHeadingDeg + turretRelNowDeg);
                absInit = true;
                validCount = 0;
            }

            // ===== Update absolute target (NO SEARCH) =====
            if (!manual) {

                if (hasTarget) validCount++;
                else validCount = 0;

                boolean outsideDeadband = Math.abs(txFilt) > txDeadbandDeg;

                if (hasTarget && outsideDeadband && validCount >= validFramesToUpdate) {

                    // IMPORTANT CHANGE:
                    // Instead of "turretAbsTarget += step",
                    // compute desired absolute direction from CURRENT turret angle + tx.
                    double turretAbsNowDeg = angleWrapDeg(robotHeadingDeg + turretRelNowDeg);

                    // Sign: +tx means tag is LEFT -> rotate turret LEFT (+)
                    double desiredAbsDeg = angleWrapDeg(turretAbsNowDeg + (txFilt * visionGain));

                    // Smoothly blend to prevent jerk/oscillation
                    turretAbsTargetDeg = lerpAngleDeg(turretAbsTargetDeg, desiredAbsDeg, lockBlend);
                }
                // else: tag lost OR inside deadband -> HOLD turretAbsTargetDeg (no scan)
            }

            // ===== Convert absolute target -> robot-relative target =====
            double turretRelTargetDeg = angleWrapDeg(turretAbsTargetDeg - robotHeadingDeg);

            // Clamp to limits
            turretRelTargetDeg = clamp(turretRelTargetDeg, minTurretDeg, maxTurretDeg);

            // Convert to ticks
            double cmdDeg = turretRelTargetDeg + turretZeroOffsetDeg;
            targetTicks = (int) Math.round(turretDir * cmdDeg * ticksperdegree);

            // ===== PID =====
            KineticState current = new KineticState(lf.getCurrentPosition(), lf.getVelocity());
            cs1.setGoal(new KineticState(targetTicks));
            double pidPower = -cs1.calculate(current);

            // Clamp power to reduce twitch
            pidPower = clamp(pidPower, -maxServoPower, +maxServoPower);

            int err = targetTicks - lf.getCurrentPosition();

            if (manual) {
                turretL.setPower(-gamepad2.left_stick_x);
                targetTicks = lf.getCurrentPosition();

                // keep field lock consistent with where driver leaves it
                turretRelNowDeg = ticksToDeg(lf.getCurrentPosition());
                turretAbsTargetDeg = angleWrapDeg(robotHeadingDeg + turretRelNowDeg);

            } else {
                // stop-zone kills buzzing and micro-oscillation
                if (Math.abs(err) <= holdZoneTicks) turretL.setPower(0);
                else turretL.setPower(pidPower);
            }

            telemetry.addData("hasTarget", hasTarget);
            telemetry.addData("txRaw", txRaw);
            telemetry.addData("txFilt", txFilt);
            telemetry.addData("validCount", validCount);
            telemetry.addData("robotHeadingDeg", robotHeadingDeg);
            telemetry.addData("turAbsTargetDeg", turretAbsTargetDeg);
            telemetry.addData("turRelNowDeg", turretRelNowDeg);
            telemetry.addData("turRelTargetDeg", turretRelTargetDeg);
            telemetry.addData("targetTicks", targetTicks);
            telemetry.addData("turretPosTicks", lf.getCurrentPosition());
            telemetry.addData("errTicks", err);
            telemetry.addData("pidPower", pidPower);
            telemetry.update();
        }
    }

    private static double ticksToDeg(int ticks) {
        return ticks / ticksperdegree;
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static double lerpAngleDeg(double aDeg, double bDeg, double t) {
        double delta = angleWrapDeg(bDeg - aDeg);
        return angleWrapDeg(aDeg + delta * t);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void configurePinpoint(GoBildaPinpointDriver pp) {
        pp.setOffsets(forwardPodY, strafePodX, ppUnit);
        pp.setEncoderResolution(podType);
        pp.setEncoderDirections(forwardDir, strafeDir);
        pp.resetPosAndIMU();
    }
}
