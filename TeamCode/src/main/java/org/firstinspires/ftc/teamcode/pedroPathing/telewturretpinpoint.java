package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "Turret Tele – Field Stabilized", group = "TeleOp")
public class telewturretpinpoint extends LinearOpMode {

    // ===================== START POSE =====================
    public static double START_X = 35.285;
    public static double START_Y = 77.683;
    public static double START_HEADING_DEG = 133.5;

    // ===================== TURRET CONFIG =====================
    public static double ticksPerDeg = 126.42;

    public static double minTurretDeg = -80;
    public static double maxTurretDeg = 80;

    public static double kP_hold = 0.02;
    public static double maxHoldPower = 0.35;

    public static double powerSlewPerSec = 1.5;
    public static double manualDeadband = 0.08;

    // Flip if direction is wrong
    public static int servoDir = -1;

    // ===================== HARDWARE =====================
    private DcMotor lf, lb, rf, rb;
    private DcMotorEx turretEnc;
    private CRServo turretL, turretR;
    private GoBildaPinpointDriver pinpoint;

    // ===================== STATE =====================
    private double robotHeadingDeg;
    private double turretRelDeg;

    private double turretAbsSetpointDeg;
    private boolean turretSetpointInitialized = false;

    private double lastCmd = 0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        // ---------- Drive ----------
        rconstants.initHardware(hardwareMap);

        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- Turret ----------
        turretL = rconstants.turretL;
        turretR = rconstants.turretR;

        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------- Pinpoint ----------
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        configurePinpoint(pinpoint);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        // ===================== DRIVE THREAD =====================
        new Thread(() -> {
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                lf.setPower((y + x + rx) / denom);
                lb.setPower((y - x + rx) / denom);
                rf.setPower((y - x - rx) / denom);
                rb.setPower((y + x - rx) / denom);
            }
        }).start();

        // ===================== TURRET LOOP =====================
        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;
            if (dt <= 0) dt = 0.02;

            // ---------- Pose ----------
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            // ---------- Turret ----------
            turretRelDeg = turretEnc.getCurrentPosition() / ticksPerDeg;

            // Initialize absolute turret heading ONCE
            if (!turretSetpointInitialized) {
                turretAbsSetpointDeg = angleWrap(robotHeadingDeg + turretRelDeg);
                turretSetpointInitialized = true;
            }

            // ---------- Manual Override ----------
            double manual = gamepad2.right_stick_x;
            boolean manualActive = Math.abs(manual) > manualDeadband;

            double cmd;

            if (manualActive) {
                cmd = clamp(manual, -1, 1);

                // Update absolute setpoint while driver moves turret
                turretAbsSetpointDeg =
                        angleWrap(robotHeadingDeg + turretRelDeg);

            } else {
                // ---------- Field-stabilized control ----------
                double desiredRelDeg =
                        angleWrap(turretAbsSetpointDeg - robotHeadingDeg);

                desiredRelDeg = clamp(desiredRelDeg, minTurretDeg, maxTurretDeg);

                double errorDeg =
                        angleWrap(desiredRelDeg - turretRelDeg);

                cmd = clamp(
                        kP_hold * errorDeg,
                        -maxHoldPower,
                        maxHoldPower
                );
            }

            // ---------- Slew ----------
            double maxDelta = powerSlewPerSec * dt;
            cmd = clamp(cmd, lastCmd - maxDelta, lastCmd + maxDelta);
            lastCmd = cmd;

            // ---------- Soft Limits ----------
            if (turretRelDeg <= minTurretDeg && cmd < 0) cmd = 0;
            if (turretRelDeg >= maxTurretDeg && cmd > 0) cmd = 0;

            turretL.setPower(servoDir * cmd);
            turretR.setPower(servoDir * cmd);

            // ---------- Telemetry ----------
            telemetry.addData("Robot Heading", robotHeadingDeg);
            telemetry.addData("Turret Rel Deg", turretRelDeg);
            telemetry.addData("Turret Abs Set", turretAbsSetpointDeg);
            telemetry.addData("Cmd", cmd);
            telemetry.update();
        }
    }

    // ===================== PINPOINT SETUP =====================
    private void configurePinpoint(GoBildaPinpointDriver pp) {
        pp.setOffsets(-5.46, -1.693, DistanceUnit.INCH);
        pp.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // IMPORTANT: reset IMU only
        pp.resetPosAndIMU();

        // Set known field pose
        pp.setPosition(new Pose2D(
                DistanceUnit.INCH,
                START_X,
                START_Y,
                AngleUnit.DEGREES,
                START_HEADING_DEG
        ));
    }

    // ===================== UTILS =====================
    private static double angleWrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
