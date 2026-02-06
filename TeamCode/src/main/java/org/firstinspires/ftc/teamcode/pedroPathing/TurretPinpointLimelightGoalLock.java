package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp(name = "Turret Pinpoint + Limelight Goal Lock")
public class TurretPinpointLimelightGoalLock extends LinearOpMode {

    // ===================== DRIVE =====================
    private DcMotor lf, lb, rf, rb;

    // ===================== TURRET =====================
    private CRServo turretL;
    private CRServo turretR;
    private DcMotorEx turretEnc; // your "turret_enc"

    private ControlSystem turretPID;

    // ===================== PINPOINT =====================
    private GoBildaPinpointDriver pinpoint;

    // ===================== LIMELIGHT =====================
    private Limelight3A limelight;

    // ===================== CONFIG =====================
    // Start pose (optional; you can keep your reset button instead)
    public static double START_X_IN = 35.285;
    public static double START_Y_IN = 77.683;
    public static double START_HEADING_DEG = 133.5;
    public static double ticks;

    // Field goal location (INCHES)
    public static double GOAL_X_IN = 0.0;
    public static double GOAL_Y_IN = 144.0;

    // Turret encoder conversion
    public static double ticksPerDegree = 126.42;

    // Turret PID (POSITION)
    public static double kP = 0.00035;
    public static double kI = 0.0000000005;
    public static double kD = 0.0000000002;

    // Limelight fusion
    // If your Limelight yaw error is actually in getTy() like your code, flip this to use TY below.
    public static boolean useTx = false;          // true = use getTx(), false = use getTy()
    public static double llAngleScale = 1.0;     // degrees of turret correction per degree of LL error
    public static double llOffsetDeg = -3.5;      // constant aim offset (degrees)
    public static double llDeadbandDeg = 0.8;    // ignore tiny jitter
    public static double llMaxAssistDeg = 12.0;  // don’t let LL swing your turret too hard

    // Auto / manual
    public static boolean autoAlign = true;
    public static double manualDeadband = 0.08;

    // Turret output direction (depends on how your servo is mounted)
    // If turret moves opposite, flip this sign.
    public static int servoDir = 1;

    // ===================== STATE (telemetry) =====================
    public static double robotX, robotY, robotHeadingDeg;
    public static double fieldAngleDeg, llErrDeg, fusedAngleDeg;
    public static double targetTicks, turretPosTicks, turretRelDeg;
    public static boolean llHasTarget;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        // --- your hardware init style ---
        rconstants.initHardware(hardwareMap);

        // Drive
        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turret servos (your telewturretblue style)
        turretL = rconstants.turretL;
        turretR = rconstants.turretR;

        // Turret encoder
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        configurePinpoint(pinpoint);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(100);
        limelight.start();

        // Turret PID
        turretPID = ControlSystem.builder()
                .posPid(kP, kI, kD)
                .build();

        // Optional: set starting pose
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                START_X_IN,
                START_Y_IN,
                AngleUnit.DEGREES,
                START_HEADING_DEG
        ));
        Thread g1 = new Thread(() -> {
            while (opModeIsActive()) {
                turretPID = ControlSystem.builder()
                        .posPid(kP,kI,kD)
                        .build();
                if(ticks/ticksPerDegree < 90 && ticks/ticksPerDegree > -90){
                    targetTicks=ticks;
                } else if(ticks/ticksPerDegree > 90){
                    targetTicks = ticksPerDegree *90;
                } else if(ticks/ticksPerDegree <-90){
                    targetTicks=ticksPerDegree*-90;
                }

                // ---------------- TURRET PID -> SERVO POWER ----------------
                KineticState current = new KineticState(
                        turretEnc.getCurrentPosition(),
                        turretEnc.getVelocity()
                );

                turretPID.setGoal(new KineticState(targetTicks));
                double out = turretPID.calculate(current);

                // your turret servo sign (you used negative in your pinpoint code)
                setTurretPower(servoDir * -out);


            }
        });
        waitForStart();
        g1.start();

        // ===================== MAIN LOOP =====================
        while (opModeIsActive()) {

            // ---------------- DRIVE ----------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            lf.setPower((y + x + rx) / denom);
            lb.setPower((y - x + rx) / denom);
            rf.setPower((y - x - rx) / denom);
            rb.setPower((y + x - rx) / denom);

            // toggle auto
            if (gamepad2.back) {
                autoAlign = !autoAlign;
                sleep(250);
            }

            // reset pose quick
            if(gamepad2.options){
                pinpoint.setPosX(136,DistanceUnit.INCH);
                pinpoint.setPosY(8.75,DistanceUnit.INCH);
                pinpoint.setHeading(180,AngleUnit.DEGREES);
            }

            // ---------------- UPDATE SENSORS ----------------
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            robotX = pose.getX(DistanceUnit.INCH);
            robotY = pose.getY(DistanceUnit.INCH);
            robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

            turretPosTicks = turretEnc.getCurrentPosition();
            turretRelDeg = turretPosTicks / ticksPerDegree;

            LLResult res = limelight.getLatestResult();
            llHasTarget = (res != null) && res.isValid();

            // ---------------- MANUAL OVERRIDE ----------------
            double manual = gamepad2.right_stick_x;
            boolean manualNow = Math.abs(manual) > manualDeadband;

            if (!autoAlign || manualNow) {
                // manual direct power
                setTurretPower(servoDir * clamp(manual, -1.0, 1.0));
                // keep PID from “snapping back” when you let go
                targetTicks = turretEnc.getCurrentPosition();
            } else {
                // ---------------- FIELD GOAL LOCK (PINPOINT) ----------------
                fieldAngleDeg = Math.toDegrees(Math.atan2(
                        GOAL_Y_IN - robotY,
                        GOAL_X_IN - robotX
                )) - robotHeadingDeg;

                fieldAngleDeg = angleWrapDeg(fieldAngleDeg);

                // ---------------- LIMELIGHT ASSIST (ONLY WHEN VISIBLE) ----------------
                llErrDeg = 0.0;

                if (llHasTarget) {
                    double raw = useTx ? res.getTx() : res.getTy();
                    raw += llOffsetDeg;

                    if (Math.abs(raw) < llDeadbandDeg) raw = 0.0;

                    // don’t let LL overpower the field solution
                    raw = clamp(raw, -llMaxAssistDeg, +llMaxAssistDeg);

                    llErrDeg = raw * llAngleScale;
                }

                // Fuse: “pinpoint points you to the goal” + “limelight centers the tag”
                fusedAngleDeg = angleWrapDeg(fieldAngleDeg + llErrDeg);

                ticks = fusedAngleDeg * ticksPerDegree;

            }

            // ---------------- TELEMETRY ----------------
            telemetry.addData("autoAlign", autoAlign);
            telemetry.addData("robot (x,y,hdg)", "%.2f, %.2f, %.1f", robotX, robotY, robotHeadingDeg);
            telemetry.addData("fieldAngleDeg", "%.2f", fieldAngleDeg);
            telemetry.addData("llHasTarget", llHasTarget);
            telemetry.addData("llErrDeg", "%.2f", llErrDeg);
            telemetry.addData("fusedAngleDeg", "%.2f", fusedAngleDeg);
            telemetry.addData("turretRelDeg", "%.2f", turretRelDeg);
            telemetry.addData("turretPosTicks", "%.0f", turretPosTicks);
            telemetry.addData("targetTicks", "%.0f", targetTicks);
            telemetry.update();
        }
    }

    // ===================== Pinpoint setup (same as your test) =====================
    private void configurePinpoint(GoBildaPinpointDriver pp) {
        pp.setOffsets(-5.46, -1.693, DistanceUnit.INCH);

        pp.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pp.resetPosAndIMU();
    }

    // ===================== Helpers =====================
    private void setTurretPower(double pwr) {
        turretL.setPower(pwr);
        turretR.setPower(pwr);
    }

    private static double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
