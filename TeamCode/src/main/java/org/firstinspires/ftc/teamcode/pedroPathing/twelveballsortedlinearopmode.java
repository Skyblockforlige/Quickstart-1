package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
@Config
@Autonomous(name = "12 Ball Sort- Blue")
public class twelveballsortedlinearopmode extends LinearOpMode {
    private Follower follower;
    public ServoImplEx transfermover;
    private DcMotorEx spindexer;
    private CRServoImplEx transfer;

    private IMU imu;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    int ballCount = 0;
    DcMotorEx turretEnc;

    boolean colorPreviouslyDetected = false;
    private Limelight3A limelight;
    public static int pattern;
    private DcMotorEx rb;
    private ControlSystem cs;
    private ControlSystem turretPID;

    public double targetx;

    public int turretOscillationDirection;
    public static double transfermoveridle = 0.6;
    public static double transfermoverscore = 0.73;
    public static double transfermoverfull = 1;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;
    private static int targetpos;
    private CRServo turretL;
    private CRServo turretR;
    private Servo hood;
    public static double targetTicksPerSecond=0;

    public static double p1=0.0009,i1=0,d1=0;
    public static double hoodtop = 0;
    public static double hoodbottom = 0.1;
    public static boolean turretmoving = true;
    public static int ball1_pos=950;
    public static int ball2_pos=950;
    public static int ball3_pos=950;
    int[] ballSlots = new int[]{0,0,0}; // 0 empty, 1 purple, 2 green
    boolean sorting = false;
    int[] sortTarget = new int[]{0,0,0};
    private CRServo turretServo;
    public static double turretp = 0.00035;
    public static double turreti = 0.0000000005;
    public static double turretd = 0.0000000002;
    public static NormalizedColorSensor colorSensor;
    ControlSystem cs1;
    int intakeBaseTarget = 0;
    boolean intakeBaseSet = false;

    boolean pendingMove = false;

    private Timer pathTimer, actionTimer, opmodeTimer,goonTimer;
    private int pathState=0;
    private final Pose startPose = new Pose(27.463, 131.821, Math.toRadians(145));

    public PathChain firstpath;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path12;
    public PathChain turn;
    public static int moveincrement = 2731;
    public static double constraint =0.6;
    public static int target = 0;
    private double transfermoverpos = 0.5;
    float[] hsv = new float[3];
    int detected = 0;
    int actual= 0;
    public static boolean spindexermoved=false;
    DistanceSensor distance;
    int target3=0;



    public void buildPaths() {
        firstpath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27.463, 131.821), new Pose(33.345, 114.100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(133.5))
                .build();

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(33.345, 114.100),
                                new Pose(75.652, 82.128),
                                new Pose(72.823, 56.850),
                                new Pose(21.500, 59.293)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133.5), Math.toRadians(180))
                .build();
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(21.500, 59.293),
                                new Pose(33.88347457627118, 64.03078265204387),
                                new Pose(29.000, 68.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(29.000, 68.5),
                                new Pose(79.400, 67.600),
                                new Pose(33.345, 114.100)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133.5))
                .build();
        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(33.345, 114.100),
                                new Pose(82.678, 80.485),
                                new Pose(29.000, 83.293)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133.5), Math.toRadians(180))
                .build();
        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(29.000, 83.293), new Pose(33.345, 114.100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133.5))
                .build();
        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(33.345, 114.100),
                                new Pose(91.5, 29.1),
                                new Pose(23.000, 35.293)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(133.5), Math.toRadians(180))
                .build();
        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(new Pose(23.000, 35.293),new Pose(59.59063156939212,30.568553737535026), new Pose(33.345, 114.100))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133.5))
                .build();
        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.345, 114.100), new Pose(35.285, 77.683))
                )
                .setConstantHeadingInterpolation(Math.toRadians(133.5))
                .build();
    }
    @Override
    public void runOpMode(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(1);
        limelight.pipelineSwitch(3);
        limelight.start();
        target3=-5000;
        buildPaths();
        //colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        //colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        //colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "cs3");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        goonTimer=new Timer();
        opmodeTimer.resetTimer();
        imu = hardwareMap.get(IMU.class, "imu");
        turretOscillationDirection = 0;
        rconstants.initHardware(hardwareMap);
        colorSensor=rconstants.colorSensor;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretServo = hardwareMap.crservo.get("turretL");

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretL=hardwareMap.crservo.get("turretL");
        hood= hardwareMap.servo.get("hood");
        transfer = hardwareMap.get(CRServoImplEx.class, "transfer");
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfermover=hardwareMap.get(ServoImplEx.class,"transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorSensor.setGain(rconstants.csgain);
        distance = (DistanceSensor) colorSensor;

        while(opModeInInit()){
            LLResult llResult = limelight.getLatestResult();
            if (llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                detected=fiducialResults.get(0).getFiducialId();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("ID:", fr.getFiducialId());
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
                telemetry.addData("Detected: ",detected);
            }
            if(detected==23){
                actual=21;
            } else if(detected==22){
                actual=23;
            } else {
                actual = 22;
            }
            telemetry.addData("Actual: ", actual);

            turretPID = ControlSystem.builder()
                    .posPid(turretp,turreti,turretd)
                    .build();
            turretPID.setGoal(new KineticState(-13000));
            KineticState current3 = new KineticState(turretEnc.getCurrentPosition());

            if (Math.abs(gamepad2.left_stick_x) < 0.05) {

                turretServo.setPower(-turretPID.calculate(current3));

            } else {
                turretServo.setPower(-gamepad2.left_stick_x);
                target = turretEnc.getCurrentPosition();
            }
            telemetry.update();
        }



        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        target=0;
        //motif = "PGP";

        transfermover.setPosition(rconstants.transfermoverscore);
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v,a,s)
                .build();
        cs1 = ControlSystem.builder()
                .posPid(p1)
                .build();
        Thread g1 = new Thread(() -> {
            while (opModeIsActive()) {
                turretPID = ControlSystem.builder()
                        .posPid(turretp,turreti,turretd)
                        .build();
                turretPID.setGoal(new KineticState(0));

                KineticState current4 = new KineticState(turretEnc.getCurrentPosition(),turretEnc.getVelocity());
                turretL.setPower(-turretPID.calculate(current4));

            }
        });
        waitForStart();
        g1.start();
        opmodeTimer.resetTimer();
        while(opModeIsActive()){


            follower.update();
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            autonomousPathUpdate();
            KineticState current2 = new KineticState(spindexer.getCurrentPosition(),spindexer.getVelocity());
            cs1.setGoal(new KineticState(target));
            spindexer.setPower(0.85*(-cs1.calculate(current2)));
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
            flywheel.setPower(cs.calculate(current1));
            telemetry.addData("sped", flywheel.getVelocity());
            telemetry.addData("power of spindexer", cs1.calculate(current2));
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("position of spindexer",spindexer.getCurrentPosition());
            telemetry.addData("target",target);
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("path state", pathState);
            telemetry.addData("Detected: ", detected);
            telemetry.update();

        }
    }

    public void autonomousPathUpdate() {
        int pos = spindexer.getCurrentPosition();
        switch (actual){
            case 21:
                //GPP
                switch (pathState) {
                    case 0:
                        transfer.setPower(1);
                        follower.followPath(firstpath);
                        targetTicksPerSecond = 1025;
                        target3=0;
                        setPathState(1);


                        //shoot 2 balls
                        break;

                    case 1:
                        follower.turnTo(Math.toRadians(133.5));
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore) && flywheel.getVelocity() >= 970) {
                            intake.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 4 * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (4 * rconstants.movespindexer - 600)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(2);

                        }

                        //shoot third ball
                        break;
                    case 2:
                        //move to begening of 1,2,3
                        if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                            follower.followPath(Path1);
                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        // READ COLOR (same hue method as teleop)
                        boolean distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>10.5) && !follower.isBusy()) {
                            setPathState(4);
                        }

                    case 4:
                        if (!follower.isBusy()) {
                            follower.setMaxPower(1);
                            target = 9*rconstants.movespindexer;
                            follower.followPath(Path2);
                            //move to shoot position
                        }
                        if (pathTimer.getElapsedTimeSeconds() > 1) {
                            setPathState(5);
                        }
                        break;
                    case 5:
                        if (!follower.isBusy()) {
                            follower.setMaxPower(1);
                            follower.followPath(Path3);
                            setPathState(6);
                            //move to shoot position
                        }
                        break;
                    case 6:
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore)) {
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 13 * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (13 * rconstants.movespindexer - 800)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(7);

                        }
                        break;
                    case 7:
                        if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                            ballCount = 0;
                            follower.followPath(Path4);

                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);

                            setPathState(8);
                        }


                        break;
                    case 8:
                         distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(9);
                        }


                        break;
                    case 9:
                        if (!follower.isBusy()) {
                            //move to shooting position for balls 4,5,6
                            follower.followPath(Path5);
                            setPathState(10);
                        }
                        break;
                    case 10:
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore)) {
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 20 * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (20 * rconstants.movespindexer - 1000)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(11);

                        }
                        break;
                    case 11:
                        if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                            ballCount = 0;
                            follower.followPath(Path6);

                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);

                            setPathState(12);
                        }
                        break;
                    case 12:
                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(13);
                        }


                        break;
                    case 13:
                        if (!follower.isBusy()) {
                            //move to shooting position for balls 4,5,6
                            follower.followPath(Path7);
                            target = 24 * rconstants.movespindexer;
                            setPathState(14);
                        }
                        break;
                    case 14:
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore)) {
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 28   * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (28 * rconstants.movespindexer - 1200)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(15);

                        }
                        break;
                    case 15:
                        if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                            follower.followPath(Path8);
                            transfermover.setPosition(rconstants.transfermoveridle);
                            //shoot ball 6
                            setPathState(-1);
                        }
                        break;
                    case 16:
                        if (!follower.isBusy()) {
                            //picks up balls 4,5,6
                            transfer.setPower(0);
                            follower.setMaxPower(0.5);
                            follower.followPath(Path9);
                            setPathState(17);
                        }
                        break;
                    case 17:

                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(18);
                        }


                        break;
                    case 18:
                        //spindexer.setPower(0.5);
                        if (!follower.isBusy()) {
                            follower.setMaxPower(1);
                            follower.followPath(Path10);
                            setPathState(19);
                        }
                        break;
                    case 19:
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore)) {
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 22 * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (22 * rconstants.movespindexer - 800)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(20);

                        }
                        break;
                    case 20:
                        if (!follower.isBusy()) {
                            follower.followPath(Path12);
                            setPathState(-1);
                        }
                    case -1:
                        stop();
                }
            case 22:
                //PGP
                switch (pathState) {
                    case 0:
                        transfer.setPower(1);
                        follower.followPath(firstpath);
                        targetTicksPerSecond = 1025;
                        target3=0;
                        setPathState(1);


                        //shoot 2 balls
                        break;

                    case 1:
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore) && flywheel.getVelocity() >= 970) {
                            intake.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 4 * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (4 * rconstants.movespindexer - 600)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(2);

                        }

                        //shoot third ball
                        break;
                    case 2:
                        //move to begening of 1,2,3
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            follower.followPath(Path1);
                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        // READ COLOR (same hue method as teleop)
                        boolean distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>10.5) && !follower.isBusy()) {
                            setPathState(4);
                        }
                        break;
                    case 4:
                        if(!follower.isBusy()) {
                            follower.setMaxPower(1);
                            follower.followPath(Path2);
                            setPathState(5);
                            //move to shoot position
                        }
                        break;
                    case 5:
                        if(!follower.isBusy()) {
                            follower.setMaxPower(1);
                            follower.followPath(Path3);
                            target=8*rconstants.movespindexer;
                            setPathState(6);
                            //move to shoot position
                        }
                        break;
                    case 6:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =11*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (11*rconstants.movespindexer-800)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(7);

                        }
                        break;
                    case 7:
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            ballCount=0;
                            follower.followPath(Path4);

                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);

                            setPathState(8);
                        }


                        break;
                    case 8:
                        // READ COLOR (same hue method as teleop)
                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(9);
                        }

                        break;
                    case 9:
                        if(!follower.isBusy()) {
                            //move to shooting position for balls 4,5,6
                            target=16*rconstants.movespindexer;
                            follower.followPath(Path5);
                            setPathState(10);
                        }
                        break;
                    case 10:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =19*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (19*rconstants.movespindexer-1000)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(11);

                        }
                        break;
                    case 11:
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            ballCount=0;
                            follower.followPath(Path6);

                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);

                            setPathState(12);
                        }
                        break;
                    case 12:
                        // READ COLOR (same hue method as teleop)
                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(13);
                        }


                        break;
                    case 13:
                        if(!follower.isBusy()) {
                            //move to shooting position for balls 4,5,6
                            follower.followPath(Path7);
                            setPathState(14);
                        }
                        break;
                    case 14:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =25*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (25*rconstants.movespindexer-1200)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(15);

                        }
                        break;
                    case 15:
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            follower.followPath(Path8);
                            transfermover.setPosition(rconstants.transfermoveridle);
                            //shoot ball 6
                            setPathState(-1);
                        }
                        break;
                    case 16:
                        if(!follower.isBusy()) {
                            //picks up balls 4,5,6
                            transfer.setPower(0);
                            follower.setMaxPower(0.5);
                            follower.followPath(Path9);
                            setPathState(17);
                        }
                        break;
                    case 17:

                        // READ COLOR (same hue method as teleop)
                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(18);
                        }


                        break;
                    case 18:
                        //spindexer.setPower(0.5);
                        if(!follower.isBusy())
                        {
                            follower.setMaxPower(1);
                            follower.followPath(Path10);
                            setPathState(19);
                        }
                        break;
                    case 19:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =22*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (22*rconstants.movespindexer-800)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(20);

                        }
                        break;
                    case 20:
                        if(!follower.isBusy()){
                            follower.followPath(Path12);
                            setPathState(-1);
                        }
                    case -1:
                        stop();

                }
            case 23:
                //PPG
                switch (pathState) {
                    case 0:
                        transfer.setPower(1);
                        follower.followPath(firstpath);
                        targetTicksPerSecond = 1025;
                        target3=0;
                        setPathState(1);


                        //shoot 2 balls
                        break;

                    case 1:
                        if (!follower.isBusy() && (transfermover.getPosition() != rconstants.transfermoverfull || transfermover.getPosition() == rconstants.transfermoverscore) && flywheel.getVelocity() >= 970) {
                            intake.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target = 4 * rconstants.movespindexer;
                        }
                        if (spindexer.getCurrentPosition() >= (4 * rconstants.movespindexer - 600)) {
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(2);

                        }

                        //shoot third ball
                        break;
                    case 2:
                        //move to begening of 1,2,3
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            follower.followPath(Path1);
                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);
                            setPathState(3);
                        }
                        break;
                    case 3:
                        // READ COLOR (same hue method as teleop)
                        boolean distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>10.5) && !follower.isBusy()) {
                            setPathState(4);
                        }
                        break;
                    case 4:
                        if(!follower.isBusy()) {
                            follower.setMaxPower(1);
                            follower.followPath(Path2);
                            setPathState(5);
                            //move to shoot position
                        }
                        break;
                    case 5:
                        if(!follower.isBusy()) {
                            follower.setMaxPower(1);
                            follower.followPath(Path3);
                            setPathState(6);
                            //move to shoot position
                        }
                        break;
                    case 6:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =10*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (10*rconstants.movespindexer-800)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(7);

                        }
                        break;
                    case 7:
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            ballCount=0;
                            follower.followPath(Path4);

                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);

                            setPathState(8);
                        }


                        break;
                    case 8:
                        // READ COLOR (same hue method as teleop)
                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove&& distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(9);
                        }

                        break;
                    case 9:
                        if(!follower.isBusy()) {
                            //move to shooting position for balls 4,5,6
                            target=14*rconstants.movespindexer;
                            follower.followPath(Path5);
                            setPathState(10);
                        }
                        break;
                    case 10:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =17*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (17*rconstants.movespindexer-1000)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(11);

                        }
                        break;
                    case 11:
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            ballCount=0;
                            follower.followPath(Path6);

                            transfermover.setPosition(rconstants.transfermoveridle);
                            intake.setPower(1);

                            setPathState(12);
                        }
                        break;
                    case 12:
                        // READ COLOR (same hue method as teleop)
                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(13);
                        }


                        break;
                    case 13:
                        if(!follower.isBusy()) {
                            //move to shooting position for balls 4,5,6
                            follower.followPath(Path7);
                            target=22*rconstants.movespindexer;
                            setPathState(14);
                        }
                        break;
                    case 14:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =25*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (25*rconstants.movespindexer-1200)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(15);

                        }
                        break;
                    case 15:
                        if(pathTimer.getElapsedTimeSeconds()>0.15) {
                            follower.followPath(Path8);
                            transfermover.setPosition(rconstants.transfermoveridle);
                            //shoot ball 6
                            setPathState(-1);
                        }
                        break;
                    case 16:
                        if(!follower.isBusy()) {
                            //picks up balls 4,5,6
                            transfer.setPower(0);
                            follower.setMaxPower(0.5);
                            follower.followPath(Path9);
                            setPathState(17);
                        }
                        break;
                    case 17:

                        distanceDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

                        // New ball enters
                        if (distanceDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                            // record color into slot memory

                            ballCount++;
                            colorPreviouslyDetected = true;

                            // schedule ONE move after short delay (no sleep in OpMode)
                            actionTimer.resetTimer();
                            pendingMove = true;
                        }

                        // reset detection when sensor no longer sees a ball
                        if (!distanceDetected) {
                            colorPreviouslyDetected = false;
                        }

                        // Execute the scheduled move exactly once
                        if (pendingMove && actionTimer.getElapsedTimeSeconds() > .15 && distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6) {
                            // absolute target based on count (never grows indefinitely)
                            target +=rconstants.movespindexer;
                            pendingMove = false;
                        }

                        // after 3 balls, move to next path state once follower done
                        if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                            setPathState(18);
                        }


                        break;
                    case 18:
                        //spindexer.setPower(0.5);
                        if(!follower.isBusy())
                        {
                            follower.setMaxPower(1);
                            follower.followPath(Path10);
                            setPathState(19);
                        }
                        break;
                    case 19:
                        if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                            intake.setPower(1);
                            transfer.setPower(1);
                            transfermover.setPosition(rconstants.transfermoverscore);
                            target =22*rconstants.movespindexer;
                        }
                        if(spindexer.getCurrentPosition()>= (22*rconstants.movespindexer-800)){
                            transfermover.setPosition(rconstants.transfermoverfull);
                            setPathState(20);

                        }
                        break;
                    case 20:
                        if(!follower.isBusy()){
                            follower.followPath(Path12);
                            setPathState(-1);
                        }
                    case -1:
                        stop();

                }


        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }





/*

        NormalizedRGBA cs1 = colorSensor1.getNormalizedColors();
        Color.colorToHSV(cs1.toColor(), hsvValues1);
        NormalizedRGBA cs2 = colorSensor2.getNormalizedColors();
        Color.colorToHSV(cs2.toColor(), hsvValues2);
        NormalizedRGBA cs3 = colorSensor3.getNormalizedColors();
        Color.colorToHSV(cs3.toColor(), hsvValues3);

        boolean idle = true;
        for (int i = 0; i < classifier.length; i++) {
            if (classifier[i] == 0) break;
            int next = (i + 1 < classifier.length) ? classifier[i + 1] : 0;
            int next2 = (i + 2 < classifier.length) ? classifier[i + 2] : 0;
            switch (motif) {
                case "PGP":
                    if (classifier[i] == 1 && next == 2 && next2 == 1) {
                        targetPurple = true;
                        idle = false;
                    } else if (classifier[i] == 1 && next == 2 && next2 == 0) {
                        targetPurple = true;
                        idle = false;
                    } else if (classifier[i] == 2 && next == 1) {
                        targetPurple = true;
                        idle = false;
                    }
                    break;
                case "PPG":
                    if (classifier[i] == 1 && next == 1 && next2 == 2) {
                        targetPurple = false;
                        idle = false;
                    } else if (classifier[i] == 1 && next == 2) {
                        targetPurple = false;
                        idle = false;
                    }
                    break;
                case "GPP":
                    if (classifier[i] == 2 && next == 1 && next2 == 1) {
                        targetPurple = true;
                        idle = false;
                    } else if (classifier[i] == 2 && next == 1) {
                        targetPurple = true;
                        idle = false;
                    }
                    break;
                default:
                    idle = true;
                    break;
            }
            if (!idle) break;
        }

        if (targetPurple && !idle) {
            if (hsvValues1[0] <= 290 && hsvValues1[0] >= 270) {
                spindexer.setPower(0);
                transfer.setPower(1);
                transfermover.setPosition(transfermoverpos);
            } else {
                if ((hsvValues2[0] >= 270 && hsvValues2[0] <= 290)) {
                    spindexer.setPower(1);
                    transfer.setPower(0);
                    transfermover.setPosition(0);
                } else if (hsvValues3[0] >= 270 && hsvValues3[0] <= 290) {
                    spindexer.setPower(-1);
                    transfer.setPower(0);
                    transfermover.setPosition(0);
                } else {
                    spindexer.setPower(0);
                    transfer.setPower(0);
                    transfermover.setPosition(0);
                }
            }
        } else if (!targetPurple && !idle) {
            if (hsvValues1[0] <= 110 && hsvValues1[0] >= 90) {
                spindexer.setPower(0);
                transfer.setPower(1);
                transfermover.setPosition(transfermoverpos);
            } else {
                if ((hsvValues2[0] >= 90 && hsvValues2[0] <= 110)) {
                    spindexer.setPower(1);
                    transfer.setPower(0);
                    transfermover.setPosition(0);
                } else if (hsvValues3[0] >= 90 && hsvValues3[0] <= 110) {
                    spindexer.setPower(-1);
                    transfer.setPower(0);
                    transfermover.setPosition(0);
                } else {
                    spindexer.setPower(0);
                    transfer.setPower(0);
                    transfermover.setPosition(0);
                }
            }
        } else if (idle) {
            spindexer.setPower(0);
            transfer.setPower(0);
            transfermover.setPosition(0);
        }


 */
    }

