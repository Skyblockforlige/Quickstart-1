package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

/**
 * Arctan-based turret alignment using your EXACT ControlSystem + KineticState PID style:
 * - Encoder: DcMotorEx "turret_enc"
 * - Actuator: CRServo "turretL"
 * - Pose source: PedroPathing follower.getPose()
 *
 * Driver override:
 * - gamepad2.left_stick_x manually drives turret servo
 * - When overriding, target = current encoder position (so it "holds" where you leave it)
 *
 * Target point for aiming (field coords):
 * - Set TARGET_X / TARGET_Y (or adjust with gamepad2 dpad to nudge)
 */
@Config
@Disabled
@TeleOp(name = "Arctan Turret Align (Your PID)", group = "Test")
public class acrtanbasedturretalignment extends LinearOpMode {

    public static double p = 0.00035, i = 0.0000000005, d = 0.0000000002;


    public static double ticksperdegree = 126.42; // ticks / deg
    public static double ticksPerRad = ticksperdegree * (180.0 / Math.PI); // ticks / rad

    // Turret limits (radians)
    //TODO: Find actual values
    public static double turretMinRad = Math.toRadians(-110);
    public static double turretMaxRad = Math.toRadians(110);

    public static double turretOffsetRad = 0.0;

    public static double angleToleranceRad = Math.toRadians(5);


    // Target (field coords, inches)
    public static double TARGET_X = 127.91598599766628;
    public static double TARGET_Y = 131.10385064177362;

    //inches per press
    public static double targetNudge = 2.0;


    private DcMotorEx turretEnc;
    private CRServo turretServo;


    private Follower follower;
    public static Pose START_POSE = new Pose(116.537, 131.821, Math.toRadians(35));


    private ControlSystem turretCS;


    public static int targetTicks = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );


        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretServo = hardwareMap.crservo.get("turretL");

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.update();
        follower.startTeleopDrive();


        turretCS = ControlSystem.builder()
                .posPid(p, i, d)
                .build();


        targetTicks = turretEnc.getCurrentPosition();
        turretCS.setGoal(new KineticState(targetTicks));

        telemetry.addLine("Ready. Press PLAY.");
        telemetry.addData("START_POSE", poseToString(START_POSE));
        telemetry.addData("Target (x,y)", "(%.1f, %.1f)", TARGET_X, TARGET_Y);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.getPose();
            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading(); // radians

            KineticState current = new KineticState(turretEnc.getCurrentPosition(), turretEnc.getVelocity());


            if (gamepad2.dpad_up)    TARGET_Y += targetNudge;
            if (gamepad2.dpad_down)  TARGET_Y -= targetNudge;
            if (gamepad2.dpad_right) TARGET_X += targetNudge;
            if (gamepad2.dpad_left)  TARGET_X -= targetNudge;


            double currentTurretRad = ticksToRad(turretEnc.getCurrentPosition());
            double desiredTurretRad = computeTurretSetpointRad(
                    robotX, robotY, robotHeading,
                    TARGET_X, TARGET_Y,
                    currentTurretRad
            );

            int autoAimTicks = radToTicks(desiredTurretRad);


            if (Math.abs(gamepad2.left_stick_x) < 0.02) {
                // Auto-aim target
                targetTicks = autoAimTicks;
                turretCS.setGoal(new KineticState(targetTicks));

                double powerCmd = -turretCS.calculate(current);

                if (nearEdgeRad(currentTurretRad) && wouldPushFurtherIntoEdge(currentTurretRad, powerCmd)) {
                    powerCmd = 0.0;
                }

                turretServo.setPower(powerCmd);

                telemetry.addData("Mode", "AUTO AIM (PID)");
                telemetry.addData("Turret Power", powerCmd);
            } else {
                // Manual power
                turretServo.setPower(-gamepad2.left_stick_x);

                targetTicks = turretEnc.getCurrentPosition();
                turretCS.setGoal(new KineticState(targetTicks));

                telemetry.addData("Mode", "MANUAL");
                telemetry.addData("Turret Power", -gamepad2.left_stick_x);
            }

            // 7) Telemetry
            telemetry.addData("Robot Pose", "(x=%.2f, y=%.2f, h=%.3f rad)", robotX, robotY, robotHeading);
            telemetry.addData("Aim Target", "(x=%.2f, y=%.2f)", TARGET_X, TARGET_Y);

            telemetry.addData("Turret Current (ticks)", turretEnc.getCurrentPosition());
            telemetry.addData("Turret Target (ticks)", targetTicks);

            telemetry.addData("Turret Current (deg)", "%.1f", Math.toDegrees(currentTurretRad));
            telemetry.addData("Turret Target (deg)", "%.1f", Math.toDegrees(desiredTurretRad));

            telemetry.update();
        }
    }


    private static double computeTurretSetpointRad(
            double robotX, double robotY, double robotHeadingRad,
            double targetX, double targetY,
            double currentTurretRad
    ) {
        // Field angle robot -> target
        double thetaField = Math.atan2(targetY - robotY, targetX - robotX);

        // Desired turret relative to robot heading
        double desired = wrapAngle(thetaField - robotHeadingRad - turretOffsetRad);

        // Choose equivalent angle (desired +/- 2π) that stays within hard limits and is closest to current
        Double best = bestAngle(desired, currentTurretRad, turretMinRad, turretMaxRad);

        // If no valid candidate exists, just hold current
        return (best == null) ? currentTurretRad : best;
    }

    private static double wrapAngle(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI; // must be -PI
        return a;
    }

    private static Double bestAngle(double desired, double current, double min, double max) {
        Double best = null;
        double bestErr = Double.POSITIVE_INFINITY;

        for (int k = -1; k <= 1; k++) {
            double candidate = desired + k * 2.0 * Math.PI;

            if (candidate < min || candidate > max) continue;

            double err = Math.abs(candidate - current);
            if (err < bestErr) {
                bestErr = err;
                best = candidate;
            }
        }
        return best;
    }


    private static double ticksToRad(int ticks) {
        return ticks / ticksPerRad;
    }

    private static int radToTicks(double rad) {
        return (int) Math.round(rad * ticksPerRad);
    }



    private static boolean nearEdgeRad(double turretRad) {
        return (turretRad < turretMinRad + angleToleranceRad) ||
                (turretRad > turretMaxRad - angleToleranceRad);
    }

    private static boolean wouldPushFurtherIntoEdge(double turretRad, double powerCmd) {
        if (turretRad < turretMinRad + angleToleranceRad) return powerCmd < 0;
        if (turretRad > turretMaxRad - angleToleranceRad) return powerCmd > 0;
        return false;
    }

    private static String poseToString(Pose p) {
        return String.format("(%.2f, %.2f, %.3f rad)", p.getX(), p.getY(), p.getHeading());
    }
}
