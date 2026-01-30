package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Config
@TeleOp
public class turrettest extends LinearOpMode {

    // ===== HARDWARE =====
    DcMotorEx turretEnc;
    CRServo turretServo;

    // ===== PEDRO =====
    Follower follower;

    // ===== CONTROL =====
    ControlSystem turretPID;

    // ===== CONSTANTS =====
    public static double START_X = 136;
    public static double START_Y = 8.75;
    public static double START_HEADING_DEG = 90;

    public static double p = 0.00035;
    public static double i = 0.0000000005;
    public static double d = 0.0000000002;

    public static double ticksPerDegree = 126.42;

    // field target (same as your code)
    public static double TARGET_X = 6;
    public static double TARGET_Y = 138;

    double targetTicks = 0;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        // ===== HARDWARE INIT =====
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretServo = hardwareMap.crservo.get("turretL");

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ===== PEDRO INIT =====
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        // ===== PID INIT (ONCE) =====


        waitForStart();

        while (opModeIsActive()) {
            turretPID = ControlSystem.builder()
                    .posPid(p, i, d)
                    .build();
            // update Pedro
            follower.update();
            Pose pose = follower.getPose();

            double robotX = pose.getX();
            double robotY = pose.getY();
            double robotHeading = pose.getHeading();

            double turretAngleRad =
                    Math.atan2(
                            TARGET_Y - robotY,
                            TARGET_X - robotX
                    ) - (robotHeading - Math.PI / 2);



            targetTicks = ticksPerDegree * Math.toDegrees(turretAngleRad);

            // manual trim
            if (gamepad2.dpad_right) {
                targetTicks += 5 * ticksPerDegree;
                sleep(200);
            }
            if (gamepad2.dpad_left) {
                targetTicks -= 5 * ticksPerDegree;
                sleep(200);
            }

            // PID update
            KineticState current =
                    new KineticState(
                            turretEnc.getCurrentPosition(),
                            turretEnc.getVelocity()
                    );

            turretPID.setGoal(new KineticState(targetTicks));

            if (Math.abs(gamepad2.left_stick_x) < 0.05) {
                turretServo.setPower(-turretPID.calculate(current));
            } else {
                turretServo.setPower(-gamepad2.left_stick_x);
                targetTicks = turretEnc.getCurrentPosition();
            }

            // ===== TELEMETRY =====
            telemetry.addData("Pedro X", robotX);
            telemetry.addData("Pedro Y", robotY);
            telemetry.addData("Pedro Heading (deg)", Math.toDegrees(robotHeading));
            telemetry.addData("Turret Pos", turretEnc.getCurrentPosition());
            telemetry.addData("Turret Target", targetTicks);
            telemetry.addData("Turret Error", targetTicks - turretEnc.getCurrentPosition());
            telemetry.update();
        }
    }
}
