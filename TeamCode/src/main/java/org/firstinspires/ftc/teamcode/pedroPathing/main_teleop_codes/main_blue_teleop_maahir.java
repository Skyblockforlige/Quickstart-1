package org.firstinspires.ftc.teamcode.pedroPathing.main_teleop_codes;

import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.random_codes_not_needed.constants_testing;

import java.util.List;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp(name="MAIN tele-Blue>")
public class main_blue_teleop_maahir extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;

    private Follower follower;
    private IMU imu;
    private double imuHeadingOffset = 0.0;

    private DcMotorEx flywheel, intake, spindexer;
    private double shooter_reverse = 1;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private Servo hood;

    public volatile Pose llPedro = new Pose(0, 0, 0);
    public volatile boolean llValid = false;

    public static volatile double spindexer_speed_shooting = constants_testing.spindexer_speed_shooting_close;
    public static double veloffset = constants_testing.veloffset_close;

    private Limelight3A limelight;

    public static NormalizedColorSensor colorSensor;
    DistanceSensor distance;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p = 0.0039, i = 0, d = 0.0000005;
    public static double v = 0.000372, a = 0.7, s = 0.0000005;
    public static double p1 = 0.0084, i1 = 0, d1 = 0.000005;
    float[] hsv = new float[3];
    int ballCount = 0;
    public static int ballshot = 0;
    boolean colorPreviouslyDetected = false;

    int[] ballSlots = new int[]{0, 0, 0};
    boolean sorting = false;
    int[] sortTarget = new int[]{0, 0, 0};

    public static double targetTicksPerSecond = 200;
    public static double ticksPerDegree = 126.42;

    public static double START_X           = 35.285;
    public static double START_Y           = 77.683;
    public static double START_HEADING_DEG = 143;

    public static double RESET_X           = 26.37;
    public static double RESET_Y           = 131.69;
    public static double RESET_HEADING_DEG = 143;

    public static double TARGET_X = 12;
    public static double TARGET_Y = 140;

    private Servo turretL;
    private CRServo turretR;
    private DcMotorEx turretEnc;

    private GoBildaPinpointDriver pinpoint;
    public static double targetTicks    = 0;
    public static double spindexerPIDspeed = 0.1;

    public static String pp = "pp";
    public static int pipelineIndex = 5;

    private boolean movedoffsetspindexer;
    private Timer currentTimer;
    public static Timer shottimer;

    private static final double METERS_TO_INCHES = 39.3701;
    private static final double FIELD_HALF_INCHES = 72.0;
    public static double Y_OFFSET_INCHES   = 117.0;
    public static double x_OFFSET_INCHES   = 3.5;
    public static double HEADING_OFFSET_DEG = 0.0;

    public static double LL_CORRECTION_THRESHOLD_INCHES = 1.0;
    public static double LL_BLEND_ALPHA          = 0.3;
    public static double IMU_HEADING_THRESHOLD_DEG = 2.0;

    // ─── helpers ────────────────────────────────────────────────────────────

    private double normalizeAngleDeg(double deg) {
        deg = deg % 360.0;
        if (deg >  180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    /** Store an offset so getImuHeading() reads desiredDeg right now. */
    private void setImuHeading(double desiredDeg) {
        imuHeadingOffset = desiredDeg
                - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Raw IMU yaw + offset = field heading in degrees. */
    private double getImuHeading() {
        return normalizeAngleDeg(
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuHeadingOffset);
    }

    private Pose limelightMT2ToPedroPose(Pose3D llPose, double imuDeg) {
        double xIn    = llPose.getPosition().x * METERS_TO_INCHES;
        double yIn    = llPose.getPosition().y * METERS_TO_INCHES;
        double pedroX = xIn + FIELD_HALF_INCHES + x_OFFSET_INCHES;
        double pedroY = yIn + FIELD_HALF_INCHES + Y_OFFSET_INCHES;
        double heading = normalizeAngleDeg(imuDeg + HEADING_OFFSET_DEG);
        return new Pose(pedroX, pedroY, Math.toRadians(heading));
    }

    public double velocityfromdistance(double distance) {
        return (((0.0000121506 * distance - 0.00507176) * distance + 0.775497) * distance
                - 45.56069) * distance + 2023.94766 - veloffset;
    }

    // ─── runOpMode ──────────────────────────────────────────────────────────

    @Override
    public void runOpMode() {
        shottimer    = new Timer();
        currentTimer = new Timer();
        telemetry    = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        constants_testing.initHardware(hardwareMap);
        lf = constants_testing.lf;
        lb = constants_testing.lb;
        rf = constants_testing.rf;
        rb = constants_testing.rb;

        elapsedtime = new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        turretL = hardwareMap.get(Servo.class, "turretL");
        turretL.setPosition(0.5);
        turretR  = constants_testing.turretR;
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pp);
        configurePinpoint(pinpoint);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
        // Align IMU offset to START_HEADING_DEG so getImuHeading() matches Pedro from loop 1
        setImuHeading(START_HEADING_DEG);

        flywheel      = constants_testing.flywheel;
        hood          = constants_testing.hood;
        intake        = constants_testing.intake;
        spindexer     = constants_testing.spindexer;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer      = constants_testing.transfer;
        transfermover = constants_testing.transfermover;
        colorSensor   = constants_testing.colorSensor;
        distance      = (DistanceSensor) colorSensor;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();

        follower.startTeleopDrive();

        int target = 0;

        Thread driveThread = new Thread(() -> {
            while (opModeIsActive()) {

                // 1. Update Pedro localizer
                follower.update();

                // 2. IMU heading correction
                double imuDeg  = getImuHeading();
                double odomDeg = Math.toDegrees(follower.getPose().getHeading());
                if (Math.abs(normalizeAngleDeg(imuDeg - odomDeg)) > IMU_HEADING_THRESHOLD_DEG) {
                    follower.setPose(new Pose(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            Math.toRadians(imuDeg)
                    ));
                }

                // 3. Heading for LL orientation
                double currentHeadingDeg = normalizeAngleDeg(
                        Math.toDegrees(follower.getPose().getHeading()));
                limelight.updateRobotOrientation(currentHeadingDeg);

                // 4. Limelight soft correction
                LLResult latest = limelight.getLatestResult();
                if (latest != null && latest.isValid()) {
                    Pose3D mt2Pose = latest.getBotpose_MT2();
                    if (mt2Pose != null) {
                        llPedro = limelightMT2ToPedroPose(mt2Pose, currentHeadingDeg);
                        llValid  = true;

                        Pose currentPose = follower.getPose();
                        double driftX = llPedro.getX() - currentPose.getX();
                        double driftY = llPedro.getY() - currentPose.getY();

                        if (Math.abs(driftX) > LL_CORRECTION_THRESHOLD_INCHES
                                || Math.abs(driftY) > LL_CORRECTION_THRESHOLD_INCHES) {
                            follower.setPose(new Pose(
                                    currentPose.getX() + LL_BLEND_ALPHA * driftX,
                                    currentPose.getY() + LL_BLEND_ALPHA * driftY,
                                    llPedro.getHeading()
                            ));
                        }
                    }
                } else {
                    llValid = false;
                }

                // 5. Turret tracking
                Pose robotPose = follower.getPose();
                double fieldAngleToTarget = Math.toDegrees(Math.atan2(
                        TARGET_Y - robotPose.getY(),
                        TARGET_X - robotPose.getX()
                ));
                double robotHeadingDeg = normalizeAngleDeg(
                        Math.toDegrees(robotPose.getHeading()));
                double fieldAngleDeg = normalizeAngleDeg(fieldAngleToTarget - robotHeadingDeg);
                double rawTicks = fieldAngleDeg * (ticksPerDegree / 10.0);

                if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
                    targetTicks += gamepad2.right_stick_x * 2.0;
                } else {
                    targetTicks = rawTicks;
                }
                double maxTicks = 90.0 * ticksPerDegree / 10.0;
                targetTicks = Math.max(-maxTicks, Math.min(maxTicks, targetTicks));

                double turretDeg = (targetTicks * 10.0) / ticksPerDegree;
                double servoPos  = 0.5 - (turretDeg / 90.0) * 0.5;
                turretL.setPosition(Math.max(0.0, Math.min(1.0, servoPos)));

                // 6. Drive
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true
                );

                // 7. Transfer
                if (gamepad2.right_trigger > 0) {
                    transfermover.setPosition(gamepad2.right_bumper
                            ? constants_testing.transfermoverfull
                            : constants_testing.transfermoverscore);
                    transfer.setPower(gamepad2.right_trigger);
                } else {
                    transfermover.setPosition(constants_testing.transfermoveridle);
                    transfer.setPower(0);
                }

                // 8. Intake with stall protection
                if (intake.getCurrent(CurrentUnit.AMPS) < 6.5) {
                    intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    currentTimer.resetTimer();
                } else if (currentTimer.getElapsedTimeSeconds() > 1.2) {
                    intake.setPower(-1);
                }

                // 9. Flywheel + hood — auto distance only
                double dist = getDistance();
                if (dist >= 50) {
                    hood.setPosition(constants_testing.hoodtop);
                    targetTicksPerSecond = velocityfromdistance(dist);
                } else {
                    hood.setPosition(constants_testing.hoodbottom);
                    targetTicksPerSecond = constants_testing.shooteridle;
                }

                cs = ControlSystem.builder().velPid(p, i, d).basicFF(v, a, s).build();
                cs.setGoal(new KineticState(0, targetTicksPerSecond));
                flywheel.setPower(cs.calculate(new KineticState(
                        flywheel.getCurrentPosition(), flywheel.getVelocity())));
            }
        });

        waitForStart();
        driveThread.start();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
            elapsedtime.reset();

            if (gamepad2.ps) {
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                target = 0;
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (ballCount == 3 && !movedoffsetspindexer) {
                target += (movespindexer / 2) - 750;
                movedoffsetspindexer = true;
            }
            if (gamepad2.left_trigger > 0.1) target += movespindexer / 2;

            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
            boolean colorDetected = distance.getDistance(DistanceUnit.CM) > 3
                    && distance.getDistance(DistanceUnit.CM) < 6;

            if (Math.abs(intake.getPower()) > 0.05 && colorDetected
                    && !colorPreviouslyDetected && ballCount < 3) {
                target += constants_testing.movespindexer;
                if (hsv[0] >= 195 && hsv[0] <= 230) ballSlots[ballCount] = 1;
                if (hsv[0] >= 140 && hsv[0] <= 180) ballSlots[ballCount] = 2;
                ballCount++;
                colorPreviouslyDetected = true;
            }
            if (!colorDetected) colorPreviouslyDetected = false;

            if (gamepad2.x) { ballCount = 0; movedoffsetspindexer = false; }

            if (gamepad2.dpad_right && ballCount == 3 && !sorting) {
                sortTarget = new int[]{1, 2, 1};
                sorting = true;
            }
            if (sorting) {
                if (!(ballSlots[0] == sortTarget[0]
                        && ballSlots[1] == sortTarget[1]
                        && ballSlots[2] == sortTarget[2])) {
                    target += constants_testing.movespindexer;
                    int temp     = ballSlots[0];
                    ballSlots[0] = ballSlots[1];
                    ballSlots[1] = ballSlots[2];
                    ballSlots[2] = temp;
                    sleep(250);
                } else {
                    sorting = false;
                }
            }

            if (getDistance() >= 110) {
                spindexer_speed_shooting = constants_testing.spindexer_speed_shooting_far;
                veloffset = constants_testing.veloffset_far;
            } else {
                spindexer_speed_shooting = constants_testing.spindexer_speed_shooting_close;
                veloffset = constants_testing.veloffset_close;
            }

            cs1 = ControlSystem.builder().posPid(p1, i1, d1).build();
            cs1.setGoal(new KineticState(target));
            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                spindexer.setPower(-spindexerPIDspeed
                        * cs1.calculate(new KineticState(spindexer.getCurrentPosition())));
            } else {
                shooter_reverse = gamepad2.left_stick_y < 0 ? -1 : 1;
                spindexer.setPower(spindexer_speed_shooting * shooter_reverse);
                target = spindexer.getCurrentPosition();
            }

            if (gamepad2.left_bumper) {
                target += constants_testing.movespindexer
                        - (target % constants_testing.movespindexer);
                sleep(300);
            }

            // Reset pose + sync IMU offset to RESET_HEADING_DEG
            if (gamepad2.right_stick_button) {
                follower.setPose(new Pose(RESET_X, RESET_Y, Math.toRadians(RESET_HEADING_DEG)));
                setImuHeading(RESET_HEADING_DEG);
            }

            Pose debugPose = follower.getPose();
            telemetry.addData("Ball Count",    ballCount);
            telemetry.addData("Loop time",     elapsedtime.toString());
            telemetry.addData("Target",        target);
            telemetry.addData("Distance (in)", String.format("%.2f in", getDistance()));
            telemetry.addData("Shooter vel",   targetTicksPerSecond);
            telemetry.addData("Pedro X (in)",  String.format("%.3f in", debugPose.getX()));
            telemetry.addData("Pedro Y (in)",  String.format("%.3f in", debugPose.getY()));
            telemetry.addData("Pedro Heading", String.format("%.2f°",
                    normalizeAngleDeg(Math.toDegrees(debugPose.getHeading()))));
            telemetry.addData("IMU Heading",   String.format("%.2f°", getImuHeading()));
            telemetry.addData("LL Valid",      llValid);
            if (llValid) {
                telemetry.addData("LL X (in)",  String.format("%.3f in", llPedro.getX()));
                telemetry.addData("LL Y (in)",  String.format("%.3f in", llPedro.getY()));
                telemetry.addData("LL Heading", String.format("%.2f°",
                        normalizeAngleDeg(Math.toDegrees(llPedro.getHeading()))));
            }
            telemetry.addData("Intake Amps",   intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    private double getDistance() {
        Pose p = follower.getPose();
        return Math.sqrt(
                Math.pow(TARGET_Y - p.getY(), 2) +
                        Math.pow(TARGET_X - p.getX(), 2));
    }

    private void configurePinpoint(GoBildaPinpointDriver pp) {
        pp.setOffsets(-5.46, -1.693, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pp.resetPosAndIMU();
        pp.setPosition(new Pose2D(
                DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING_DEG));
    }
}