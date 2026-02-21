package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
@Config
@Autonomous(name = "farautodiffblue", group = "Examples")
public class farautodiffblue extends OpMode {
    private Follower follower;
    public ServoImplEx transfermover;
    private DcMotorEx spindexer;
    private CRServoImplEx transfer;
    public static double errAlpha = 1.5;
    public static double deadbandDeg = 2.0;


    public static double forwardPodY = -5.46;
    public static double strafePodX = -1.693;
    private IMU imu;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    public static double spindexerspeed = 0.1;
    int ballCount = 0;
    boolean colorPreviouslyDetected = false;
    private Limelight3A limelight;
    private DcMotorEx rb;
    private ControlSystem cs;

    public double targetx;
    public static double turrettarget=290;

    public static double turretp = 0.002;
    public static double turreti = 0;
    public static double turretd = 0.00000005;
    public static double turretv = 0.0000372;
    public static double turreta = 0.007;
    public static double turrets = 0.05;
    private ControlSystem turretPID;

    public int turretOscillationDirection;
    public static double transfermoveridle = 0.6;
    public static double transfermoverscore = 0.73;
    public static double transfermoverfull = 1;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;
    private static int targetpos;
    private CRServo turretL;
    private CRServo turretR;

    public boolean first = false;
    public boolean second = true;
    public boolean third_1 = true;

    private Servo hood;
    public static double targetTicksPerSecond=0;
    public boolean switch_case=false;
    public boolean check_follower=false;
    public boolean switchcase1=false;

    public static double p1=0.0009,i1=0,d1=0;
    public static double hoodtop = 0;
    public static double hoodbottom = 0.1;
    public static int ball1_pos=950;
    public static int ball2_pos=950;
    public static int ball3_pos=950;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
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
    private final Pose startPose = new Pose(56.000, 8, Math.toRadians(90));

    public PathChain firstpath;
    public PathChain go_first_back;

    public PathChain go_second;
    public PathChain shoot2_row;
    DcMotorEx turretEnc;



    public static int moveincrement = 2731;
    public static double constraint =0.6;
    public static int target = 0;
    private double transfermoverpos = 0.5;
    float[] hsv = new float[3];
    public static boolean spindexermoved=false;
    DistanceSensor distance;

    public static double kP_hold = 0.02;
    public static double kP_track = 0.015;

    public static double maxHoldPower = 0.3;
    public static double maxTrackPower = 0.25;

    public static double maxTurretDeg = 60;
    public static double minTurretDeg = -60;

    public static int pipelineIndex = 1;
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
    public static double tyOffsetDeg = -5;
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
    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(56.000, 19.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 19.000),
                                new Pose(52.72646657571624, 27.77070017777499),
                                new Pose(20.128, 23.839)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.128, 23.839),

                                new Pose(56.000, 19.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 9.000),

                                new Pose(20, 1)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(200))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(20, 1),

                                new Pose(56.000, 19.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(200), Math.toRadians(90))

                .build();
        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 19.000),

                                new Pose(31.014, 14.421)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();
    }


    @Override
    public void init() {
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
        imu = hardwareMap.get(IMU.class, "imu");
        turretOscillationDirection = 0;
        rconstants.initHardware(hardwareMap);
        colorSensor=rconstants.colorSensor;
        turretL = hardwareMap.crservo.get("turretL");
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        hood= hardwareMap.servo.get("hood");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        turretL.setPower(0);
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
        colorSensor.setGain(2.7f);
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
        hood.setPosition(rconstants.hoodtop);
        limelight = rconstants.limelight;
        limelight.pipelineSwitch(pipelineIndex);
        limelight.setPollRateHz(100);
        limelight.start();


    }
    @Override
    public void init_loop(){
        turretPID = ControlSystem.builder()
                .posPid(turretp,turreti,turretd)
                .basicFF(turretv,turreta,turrets)
                .build();
        turretPID.setGoal(new KineticState(turrettarget));
        KineticState current4 = new KineticState(turretEnc.getCurrentPosition()/10.0);
        turretL.setPower(-turretPID.calculate(current4));
    }
    public void start(){
        opmodeTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        int pos = spindexer.getCurrentPosition();
        switch (pathState) {
            case 0:
                //offset go back on spindexer
                transfermover.setPosition(rconstants.transfermoverscore);
                transfer.setPower(1);
                follower.followPath(Path1);
                spindexerspeed=0.1;
                targetTicksPerSecond=rconstants.shootfar-10;

                setPathState(1);


                //shoot 2 balls
                break;
            case 1:
                /*if(!follower.isBusy()&&flywheel.getVelocity()>targetTicksPerSecond-20&!first){
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    transfer.setPower(1);
                    target =2*rconstants.movespindexer;
                    first=true;
                    second=false;
                }
                if(flywheel.getVelocity()>targetTicksPerSecond-20&spindexer.getCurrentPosition()>= (2*rconstants.movespindexer-200)&!second){
                    target=3*rconstants.movespindexer;
                    second=true;
                    third_1=false;
                }
                if(flywheel.getVelocity()>targetTicksPerSecond-20&spindexer.getCurrentPosition()>= (3*rconstants.movespindexer-200)&!third_1) {
                    transfermover.setPosition(rconstants.transfermoverfull);
                    third_1=true;
                    setPathState(2);
                }*/
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)&&flywheel.getVelocity()>=1500){
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =4*rconstants.movespindexer;
                    spindexerspeed=0.5;
                }
                if(spindexer.getCurrentPosition()>= (4*rconstants.movespindexer-400)&&flywheel.getVelocity()>=1500){
                    spindexerspeed=1;
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(2);
                }
                //shoot third bal
                break;
            case 2:
                //move to begening of 1,2,3
                if(pathTimer.getElapsedTimeSeconds()>0.5) {
                    spindexerspeed=1;
                    follower.followPath(Path2);
                    transfermover.setPosition(rconstants.transfermoveridle);
                    setPathState(3);

                }
                break;
            case 3:
                // READ COLOR (same hue method as teleop)
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

                float hue = hsv[0];
                boolean isPurple = (hue > 200 && hue < 300);
                boolean isGreen  = (hue > 95  && hue < 200);
                boolean colorDetected = (isPurple || isGreen);

                // New ball enters
                if (colorDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                    // record color into slot memory
                    if (isPurple) ballSlots[ballCount] = 1;
                    if (isGreen)  ballSlots[ballCount] = 2;

                    ballCount++;
                    colorPreviouslyDetected = true;

                    // schedule ONE move after short delay (no sleep in OpMode)
                    actionTimer.resetTimer();
                    pendingMove = true;
                }

                // reset detection when sensor no longer sees a ball
                if (!colorDetected) {
                    colorPreviouslyDetected = false;
                }

                // Execute the scheduled move exactly once
                if (pendingMove && actionTimer.getElapsedTimeSeconds() > .05 && distance.getDistance(DistanceUnit.CM)>4.5 && distance.getDistance(DistanceUnit.CM)<6) {
                    // absolute target based on count (never grows indefinitely)
                    target +=rconstants.movespindexer;
                    pendingMove = false;
                }

                // after 3 balls, move to next path state once follower done
                if ((ballCount >=3||pathTimer.getElapsedTimeSeconds()>3.5)) {
                    setPathState(4);
                }

                break;
            case 4:
               /*&&spindexer.getCurrentPosition()%rconstants.movespindexer>=-500 &&spindexer.getCurrentPosition()%rconstants.movespindexer<=500*/
                    follower.followPath(Path3);
                    spindexerspeed=0.2;
                    /*spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                   target=0;*/
                    setPathState(5);
                    //move to shoot position

                break;
            case 5:
                // READ COLOR (same hue method as teleop)
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)&&flywheel.getVelocity()>=1500){
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =8*rconstants.movespindexer;
                    spindexerspeed=1;

                }
                if(spindexer.getCurrentPosition()>= (8*rconstants.movespindexer-800)&&flywheel.getVelocity()>=1500){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    spindexerspeed=1;
                    setPathState(6);

                }

                break;
            case 6:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>0.15) {
                    follower.followPath(Path4);
                    transfermover.setPosition(rconstants.transfermoveridle);
                    setPathState(7);
                    //move to shoot position
                }
                break;
            case 7:
                // READ COLOR (same hue method as teleop)
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

                hue = hsv[0];
                isPurple = (hue > 200 && hue < 300);
                isGreen  = (hue > 95  && hue < 200);
                colorDetected = (isPurple || isGreen);

                // New ball enters
                if (colorDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                    // record color into slot memory
                    if (isPurple) ballSlots[ballCount] = 1;
                    if (isGreen)  ballSlots[ballCount] = 2;

                    ballCount++;
                    colorPreviouslyDetected = true;

                    // schedule ONE move after short delay (no sleep in OpMode)
                    actionTimer.resetTimer();
                    pendingMove = true;
                }

                // reset detection when sensor no longer sees a ball
                if (!colorDetected) {
                    colorPreviouslyDetected = false;
                }

                // Execute the scheduled move exactly once
                if (pendingMove && actionTimer.getElapsedTimeSeconds() > .05 && distance.getDistance(DistanceUnit.CM)>4.5 && distance.getDistance(DistanceUnit.CM)<6) {
                    // absolute target based on count (never grows indefinitely)
                    target +=rconstants.movespindexer;
                    pendingMove = false;
                }

                // after 3 balls, move to next path state once follower done
                if ((ballCount >=3||pathTimer.getElapsedTimeSeconds()>3.5)) {
                    setPathState(8);
                }
                break;
            case 8:
                    follower.followPath(Path5);
                    spindexerspeed=0.2;
                    spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target=0;
                    setPathState(9);
                    //move to shoot position

                break;
            case 9:
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)&&flywheel.getVelocity()>=1500){
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =4*rconstants.movespindexer;
                    spindexerspeed=1;

                }
                if(spindexer.getCurrentPosition()>= (4*rconstants.movespindexer-500)&&flywheel.getVelocity()>=1500){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    spindexerspeed=1;
                    if(opmodeTimer.getElapsedTimeSeconds()>=27.5) {
                        setPathState(10);
                    } else{
                        setPathState(6);

                    }
                }
                break;
            case 10:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>0.1){
                    transfermover.setPosition(rconstants.transfermoveridle);
                    follower.followPath(Path6);
                    setPathState(-1);
                }
                break;
            case 11:
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                    intake.setPower(1);
                    transfer.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =10*rconstants.movespindexer;
                }
                if(spindexer.getCurrentPosition()>= (10*rconstants.movespindexer-800)){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(-1);

                }
                break;
            case -1:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    transfermover.setPosition(rconstants.transfermoveridle);
                    stop();

                }
                break;
            /*case 8:
                ballCount=0;
                setPathState(9);

                break;
            case 9:
                //move to beginning of balls 4,5,6
                if(pathTimer.getElapsedTimeSeconds()>0.25) {
                    follower.followPath(Path5);

                    transfermover.setPosition(rconstants.transfermoveridle);
                    intake.setPower(1);

                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    //picks up balls 4,5,6
                    transfer.setPower(0);
                    follower.setMaxPower(0.5);
                    follower.followPath(Path6);
                    setPathState(11);
                }
                break;
            case 11:


                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

                hue = hsv[0];
                isPurple = (hue > 200 && hue < 300);
                isGreen  = (hue > 95  && hue < 200);
                colorDetected = (isPurple || isGreen);

                // New ball enters
                if (colorDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                    // record color into slot memory
                    if (isPurple) ballSlots[ballCount] = 1;
                    if (isGreen)  ballSlots[ballCount] = 2;

                    ballCount++;
                    colorPreviouslyDetected = true;

                    // schedule ONE move after short delay (no sleep in OpMode)
                    actionTimer.resetTimer();
                    pendingMove = true;
                }

                // reset detection when sensor no longer sees a ball
                if (!colorDetected) {
                    colorPreviouslyDetected = false;
                }

                // Execute the scheduled move exactly once
                if (pendingMove && actionTimer.getElapsedTimeSeconds() > .05 && distance.getDistance(DistanceUnit.CM)>4 && distance.getDistance(DistanceUnit.CM)<6) {
                    // absolute target based on count (never grows indefinitely)
                    target +=rconstants.movespindexer;
                    pendingMove = false;
                }

                // after 3 balls, move to next path state once follower done
                if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>3.5) && !follower.isBusy()) {
                    setPathState(12);
                }

                break;
            case 12:
                if(!follower.isBusy()) {
                    //move to shooting position for balls 4,5,6
                    follower.setMaxPower(1);
                    follower.followPath(Path7);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    //shoot balls 4


                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                    intake.setPower(1);
                    transfer.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                    target =16*rconstants.movespindexer;
                }
                if(spindexer.getCurrentPosition()>= (16*rconstants.movespindexer-600)){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    setPathState(15);

                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds()>0.25) {
                    follower.followPath(Path8);
                    transfermover.setPosition(rconstants.transfermoveridle);
                    //shoot ball 6
                    setPathState(16);
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
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

                hue = hsv[0];
                isPurple = (hue > 200 && hue < 300);
                isGreen  = (hue > 95  && hue < 200);
                colorDetected = (isPurple || isGreen);

                // New ball enters
                if (colorDetected && !colorPreviouslyDetected && ballCount < 3 && !pendingMove) {

                    // record color into slot memory
                    if (isPurple) ballSlots[ballCount] = 1;
                    if (isGreen)  ballSlots[ballCount] = 2;

                    ballCount++;
                    colorPreviouslyDetected = true;

                    // schedule ONE move after short delay (no sleep in OpMode)
                    actionTimer.resetTimer();
                    pendingMove = true;
                }

                // reset detection when sensor no longer sees a ball
                if (!colorDetected) {
                    colorPreviouslyDetected = false;
                }

                // Execute the scheduled move exactly once
                if (pendingMove && actionTimer.getElapsedTimeSeconds() > .05 && distance.getDistance(DistanceUnit.CM)>4 && distance.getDistance(DistanceUnit.CM)<6) {
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

             */

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        turretPID = ControlSystem.builder()
                .posPid(turretp,turreti,turretd)
                .basicFF(turretv,turreta,turrets)
                .build();
        turretPID.setGoal(new KineticState(turrettarget));
        KineticState current3 = new KineticState(turretEnc.getCurrentPosition()/10.0);
        turretL.setPower(-turretPID.calculate(current3));
       /* long lastNanos = System.nanoTime();
        // start in AUTO (not manual)
        turretMode = TurretMode.IDLE;


            long now = System.nanoTime();
            double dt = (now - lastNanos) / 1e9;
            lastNanos = now;
            if (dt <= 0) dt = 0.02;

            // Always update pinpoint + heading


            // Turret relative angle from encoder
            turretRelDeg = turretEnc.getCurrentPosition() / ticksPerDeg;

            // Limelight read
            LLResult res = limelight.getLatestResult();
            hasTarget = (res != null) && res.isValid();

            // ===================== FIX: APPLY TY OFFSET HERE =====================
            tyRaw = hasTarget ? (res.getTy() + tyOffsetDeg) : 0.0;

            // Filter TY
            double alpha = clamp(errAlpha, 0.0, 1.0);
            //tyFilt = (1.0 - alpha) * tyFilt + alpha * tyRaw;
            tyFilt = tyRaw;
            // Manual override only while stick is moved; release -> go back to auto
            double manualStick = -gamepad2.right_stick_x;
            boolean manualNow = Math.abs(manualStick) > manualDeadband;

            // Update lastKnownAbs direction when tag is well-centered
            if (hasTarget && Math.abs(tyFilt) <= lastKnownSaveWindowDeg) {
                lastKnownAbsDeg = angleWrapDeg(robotHeadingDeg + turretRelDeg);
                haveLastKnown = true;
            }*/

            // If manual: drive turret directly, but ALSO keep lastKnownAbs tracking turret direction
           /* if (manualNow) {
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

            }*/

            // Not manual => AUTO
            /*relUnclampedNeeded = 0.0;
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
*/
            // Command power
            /*double cmdPower;

            if (turretMode == TurretMode.TRACK) {
                // drive TY to 0
                if (Math.abs(tyFilt) <= deadbandDeg) {
                    cmdPower = 0.0;
                } else {
                    cmdPower = -kP_track * tyFilt;
                    cmdPower = clamp(cmdPower, -maxTrackPower, +maxTrackPower);
                }
            }*/ /*else if (turretMode == TurretMode.HOLD) {
                    cmdPower = kP_hold * holdErrDeg;
                    cmdPower = clamp(cmdPower, -maxHoldPower, +maxHoldPower);*/
            /*else {
                cmdPower = 0.0;
                edgePauseTimer = 0.0;
                sweepTargetDeg = Double.NaN;
            }*/
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
           /* if (turretRelDeg <= minTurretDeg && cmdPower < 0) cmdPower = 0.0;
            if (turretRelDeg >= maxTurretDeg && cmdPower > 0) cmdPower = 0.0;

            // Slew limit (smooth)
            double maxDelta = powerSlewPerSec * dt;
            cmdPower = clamp(cmdPower, lastTurretCmdPower - maxDelta, lastTurretCmdPower + maxDelta);
            lastTurretCmdPower = cmdPower;

            turretOut = servoDir * cmdPower;
            setTurretPower(turretOut);*/


        /*YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
        }
        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
        }

        // Turret code - detects AprilTag and goes to its location


        if (llResult != null) {
            targetx = llResult.getTy();
            telemetry.addData("targetx", llResult.getTx());

            if (targetx >= 4.5) {
                // not necesary but makes it move exactly one degree
                turretL.setPower(0.2);
                //turretR.setPower(0.2);
                turretOscillationDirection = 0;
                //switch to negative and make other postive if goes wrong direction
            } else if (targetx <= -0.5) {
                turretL.setPower(-0.2);
                //turretR.setPower(-0.2);
                turretOscillationDirection = 1;
                turretL.setPower(0);
            }
        } else{
            if(turretOscillationDirection == 0){
                //turretL.setPower(-0.1);
                //turretR.setPower(-0.1);
                turretOscillationDirection=1;
            } else{
                //turretL.setPower(0.1);
                //turretR.setPower(0.1);
                turretOscillationDirection=0;
            }
        }





        telemetry.addData("Xseen", llResult.getTx());
        telemetry.addData("turretOscillationDirection", turretOscillationDirection);
        telemetry.addData("Tx", llResult.getTx());
        telemetry.addData("distance",distance);
        telemetry.addData("Ty", llResult.getTy());
        telemetry.addData("Ta", llResult.getTa());
        telemetry.update();*/
        colorSensor.getNormalizedColors();
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

        autonomousPathUpdate();
        KineticState current2 = new KineticState(spindexer.getCurrentPosition(),spindexer.getVelocity());
        cs1.setGoal(new KineticState(target));
        spindexer.setPower(spindexerspeed*(-cs1.calculate(current2)));
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


    // ===================== Turret power helper =====================
    private void setTurretPower(double pwr) {
        turretL.setPower(pwr);
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
