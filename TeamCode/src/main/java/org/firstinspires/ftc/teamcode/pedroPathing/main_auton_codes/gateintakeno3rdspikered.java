package org.firstinspires.ftc.teamcode.pedroPathing.main_auton_codes;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.random_codes_not_needed.constants_testing;
import org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes.rconstants;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
@Config
@Autonomous(name = "Gate Intake Auto NO 3rd SPIKE - RED")
public class gateintakeno3rdspikered extends OpMode {
    private Follower follower;
    public ServoImplEx transfermover;
    private DcMotorEx spindexer;
    private List<LynxModule> allHubs;

    private CRServoImplEx transfer;

    private IMU imu;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    int ballCount = 0;
    boolean colorPreviouslyDetected = false;
    private Limelight3A limelight;
    private DcMotorEx rb;
    private ControlSystem cs;

    public double targetx;

    public int turretOscillationDirection;
    public static double transfermoveridle = 0.6;
    public static double transfermoverscore = 0.73;
    public static double transfermoverfull = 1;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;

    private static int targetpos;
    private CRServo turretR;
    private Servo hood;
    public static double targetTicksPerSecond=0;

    public static double p1 = 0.0084, i1 = 0, d1 = 0.000005;
    public static double hoodtop = 0;
    public static double hoodbottom = 0.1;
    public static int ball1_pos=950;
    public static int ball2_pos=950;
    public static int ball3_pos=950;
    int[] ballSlots = new int[]{0,0,0}; // 0 empty, 1 purple, 2 green
    boolean sorting = false;
    int[] sortTarget = new int[]{0,0,0};
    public static NormalizedColorSensor colorSensor;
    ControlSystem cs1;
    int intakeBaseTarget = 0;
    boolean intakeBaseSet = false;

    boolean pendingMove = false;

    private Timer pathTimer, actionTimer, opmodeTimer,goonTimer;
    private int pathState=0;
    public static double turretPos = 0.23;
    private final Pose startPose = new Pose(27.463, 131.821, Math.toRadians(143)).mirror();

    public PathChain firstpath;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    private Timer currentTimer;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path12;
    public PathChain Path11;
    public PathChain Path67;
    public PathChain Path68;
    public static int moveincrement = 2731;
    public static double constraint =0.6;
    public static int target = 0;
    private double transfermoverpos = 0.5;
    public Servo turretL;
    float[] hsv = new float[3];
    public static boolean spindexermoved=false;
    DistanceSensor distance;


    public void buildPaths() {
        Path1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(27.463, 131.821).mirror(),
                                new Pose(45.005, 112.859).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .setBrakingStrength(0.9)
                .setTValueConstraint(0.85)
                .build();

        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(43.796, 99.812).mirror(),
                                new Pose(63, 56.8).mirror(),
                                new Pose(20.500, 60.100).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setTValueConstraint(0.85)
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.500, 60.100).mirror(),
                                new Pose(53.500, 56.800).mirror(),
                                new Pose(59.382, 87.557).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setTValueConstraint(0.85)
                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.382, 87.557).mirror(),
                                new Pose(52.782, 51.290).mirror(),
                                new Pose(17.702151898734183, 60.66577215189874).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(28))
                .setTValueConstraint(0.85)
                .build();

        /*Path5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(21.530, 63.124),
                                new Pose(34.157704697986574,58.29684899328858),
                                new Pose(22.141114093959725, 56.75828859060402)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(150))
                .build();*/


        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(19.530, 62.124).mirror(),
                                new Pose(52.41237583892618,65.00157382550336).mirror(),
                                new Pose(59.382, 87.557).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                .setTValueConstraint(0.85)
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(59.382, 87.557).mirror(),
                                new Pose(23.000, 87.557).mirror()
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setTValueConstraint(0.85)
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(23.000, 87.557).mirror(),
                                new Pose(59.000, 121.300).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTValueConstraint(0.85)
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.382, 85.557).mirror(),
                                new Pose(72.191, 33.044).mirror(),
                                new Pose(21.000, 35.293).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setTValueConstraint(0.85)
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(21.000, 35.293),
                                new Pose(59.000, 114.300)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .setTValueConstraint(0.85)
                .build();
        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(59.382, 85.557).mirror(),
                                new Pose(56.700, 122.975).mirror()
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setTValueConstraint(0.85)
                .build();
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        //colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        //colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        //colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "cs3");
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        goonTimer=new Timer();
        currentTimer=new Timer();
        opmodeTimer.resetTimer();
        imu = hardwareMap.get(IMU.class, "imu");
        turretOscillationDirection = 0;
        rconstants.initHardware(hardwareMap);
        colorSensor=rconstants.colorSensor;
        //turretR = hardwareMap.crservo.get("turretR");
        hood= hardwareMap.servo.get("hood");
        turretL=hardwareMap.servo.get("turretL");
        // limelight = hardwareMap.get(Limelight3A.class, "limelight");
        transfer = hardwareMap.get(CRServoImplEx.class, "transfer");
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfermover=hardwareMap.get(ServoImplEx.class,"transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //limelight.pipelineSwitch(1);
        colorSensor.setGain(rconstants.csgain);
        distance = (DistanceSensor) colorSensor;


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
        hood.setPosition(constants_testing.hoodbottom);
    }
    @Override
    public void start(){
        opmodeTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        int pos = spindexer.getCurrentPosition();
        switch (pathState) {
            case 0:
                //offset go back on spindexer
                transfer.setPower(1);
                intake.setPower(0);
                follower.followPath(Path1);
                targetTicksPerSecond=1100;
                turretPos=0.23;
                hood.setPosition(constants_testing.hoodbottom);
                setPathState(1);


                //shoot 2 balls
                break;
            case 1:

                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)&&flywheel.getVelocity()>=1050){

                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =4*rconstants.movespindexer;
                }
                if(spindexer.getCurrentPosition()>= (4*rconstants.movespindexer-600)){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(2);

                }


                //shoot third ball
                break;
            case 2:
                //move to begening of 1,2,3
                if(pathTimer.getElapsedTimeSeconds()>0.15) {
                    follower.followPath(Path2);
                    targetTicksPerSecond=1220;
                    transfer.setPower(-1);
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoveridle);

                    hood.setPosition(constants_testing.hoodtop);
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
                if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5)) {
                    turretPos=0.17;
                    intake.setPower(0);
                    setPathState(4);
                }
                break;
            case 4:
                if(follower.getCurrentPath().isAtParametricEnd()) {
                    follower.setMaxPower(1);
                    follower.followPath(Path3);
                    setPathState(5);
                    //move to shoot position
                }
                break;
            case 5:
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){

                    transfer.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =12*rconstants.movespindexer;
                }
                if(spindexer.getCurrentPosition()>= (12*rconstants.movespindexer-800)){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(6);

                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {
                    ballCount=0;
                    follower.followPath(Path4);
                    transfer.setPower(-1);
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoveridle);



                    setPathState(7);
                }
                break;
            case 7:
                setPathState(8);
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
                if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>4.5)) {
                    intake.setPower(0);
                    setPathState(9);

                }
                break;
            case 9:
                if(follower.getCurrentPath().isAtParametricEnd()) {

                    //move to shooting position for balls 4,5,6
                    follower.followPath(Path6);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){

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
                    follower.followPath(Path4);
                    transfer.setPower(-1);
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoveridle);



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
                if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>4.5)) {
                    intake.setPower(0);
                    setPathState(13);
                    targetTicksPerSecond=1220;
                }

                break;
            case 13:
                if(follower.getCurrentPath().isAtParametricEnd()) {
                    //move to shooting position for balls 4,5,6

                    follower.followPath(Path6);
                    setPathState(14);
                }
                break;
            case 14:
                if(opmodeTimer.getElapsedTimeSeconds()>29){
                    setPathState(15);
                }
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){

                    transfer.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =26*rconstants.movespindexer;
                }
                if(spindexer.getCurrentPosition()>= (26*rconstants.movespindexer-1200)){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(15);

                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds()>0.15) {
                    follower.followPath(Path7);
                    ballCount=0;
                    transfermover.setPosition(rconstants.transfermoveridle);
                    transfer.setPower(-1);
                    intake.setPower(1);
                    //shoot ball 6
                    setPathState(16);
                }
                break;
            case 16:
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
                if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>2)) {
                    turretPos=0.08;
                    intake.setPower(0);
                    hood.setPosition(constants_testing.hoodbottom);
                    targetTicksPerSecond=1120;
                    setPathState(17);
                }

                break;
            case 17:

                follower.followPath(Path8);
                setPathState(18);
                break;
            case 18:
                //spindexer.setPower(0.5);
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){

                    transfer.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =34*rconstants.movespindexer;
                }
                if(spindexer.getCurrentPosition()>= (34*rconstants.movespindexer-800)){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(19);

                }
                break;
            case 19:
                if(pathTimer.getElapsedTimeSeconds()>0.3) {
                    transfermover.setPosition(rconstants.transfermoveridle);
                    transfer.setPower(0);
                    setPathState(-1);
                }
                break;
            case 20:
                if(follower.getCurrentPath().isAtParametricEnd()){
                    follower.followPath(Path11);
                    setPathState(-1);
                }
            case -1:
                stop();

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
            for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        follower.update();
        if(flywheel.getVelocity()<(targetTicksPerSecond-90)){
            flywheel.setPower(1);
        } else{
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
            flywheel.setPower(cs.calculate(current1));
        }
        //hood.setPosition(constants_testing.hoodtop);
        /*if(intake.getCurrent(CurrentUnit.AMPS)<5) {
            intake.setPower(1);
            currentTimer.resetTimer();
        } else if(currentTimer.getElapsedTimeSeconds()>1.2){
            intake.setPower(-1);
        } else{
            intake.setPower(1);
        } */
        turretL.setPosition(turretPos);
        colorSensor.getNormalizedColors();
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

        autonomousPathUpdate();
        KineticState current2 = new KineticState(spindexer.getCurrentPosition(),spindexer.getVelocity());
        cs1.setGoal(new KineticState(target));
        spindexer.setPower(Range.clip(-0.6 * cs1.calculate(current2),-0.6,0.6));
        /*cs.setGoal(new KineticState(0,targetTicksPerSecond));
        KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
        flywheel.setPower(cs.calculate(current1));*/
        telemetry.addData("sped", flywheel.getVelocity());
        telemetry.addData("power of spindexer", cs1.calculate(current2));
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("position of spindexer",spindexer.getCurrentPosition());
        telemetry.addData("target",target);
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
        telemetry.addData("path state", pathState);
        telemetry.update();




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
}
