package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
@Config
@Disabled
@Autonomous(name = "3 ball red - w open source", group = "Examples")
public class randoauton1 extends OpMode {
    private Follower follower;
    public ServoImplEx transfermover;
    private DcMotorEx spindexer;
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
    private CRServo turretL;
    private CRServo turretR;
    private Servo hood;
    public static double targetTicksPerSecond=0;

    public static double p1=0.0009,i1=0,d1=0;
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
    public static int moveincrement = 2731;
    public static double constraint =0.6;
    public static int target = 0;
    private double transfermoverpos = 0.5;
    float[] hsv = new float[3];
    public static boolean spindexermoved=false;
    DistanceSensor distance;


    public void buildPaths() {
        firstpath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27.463, 131.821), new Pose(27.475, 115.940))
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                .build();

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(27.475, 115.940), new Pose(14.419, 103.859))
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
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
        opmodeTimer.resetTimer();
        imu = hardwareMap.get(IMU.class, "imu");
        turretOscillationDirection = 0;
        rconstants.initHardware(hardwareMap);
        colorSensor=rconstants.colorSensor;
        turretL = hardwareMap.crservo.get("turretL");
        turretR = hardwareMap.crservo.get("turretR");
        hood= hardwareMap.servo.get("hood");
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
        transfermover.setPosition(rconstants.transfermoveridle);
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v,a,s)
                .build();
        cs1 = ControlSystem.builder()
                .posPid(p1)
                .build();
        hood.setPosition(0.4);
    }

    public void autonomousPathUpdate() {

        int pos = spindexer.getCurrentPosition();
        switch (pathState) {
            case 0:
                //offset go back on spindexer

                transfer.setPower(1);
                follower.followPath(firstpath);
                targetTicksPerSecond=1000;

                setPathState(1);


                //shoot 2 balls
                break;
            case 1:
                if(!follower.isBusy()&&(transfermover.getPosition()!=rconstants.transfermoverfull||transfermover.getPosition()==rconstants.transfermoverscore)){
                    intake.setPower(1);
                    transfermover.setPosition(rconstants.transfermoverscore);
                }
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>3&& pathTimer.getElapsedTimeSeconds()<4 &&flywheel.getVelocity()>=1150) {
                    target =2*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.3&&pathTimer.getElapsedTimeSeconds()<4.6) {
                    target =3*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.6&&pathTimer.getElapsedTimeSeconds()<4.8) {
                    target =4*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.8&&pathTimer.getElapsedTimeSeconds()<5.1){
                    transfermover.setPosition(rconstants.transfermoverfull);

                }
                if(pathTimer.getElapsedTimeSeconds()>5.1){
                    setPathState(2);
                }

                //shoot third ball
                break;
            case 2:
                //move to begening of 1,2,3
                follower.followPath(Path1);
                setPathState(3);
                break;
            case 3:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
            case 4:
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
                if (pendingMove && distance.getDistance(DistanceUnit.CM)>4.5 && distance.getDistance(DistanceUnit.CM)<6) {
                    // absolute target based on count (never grows indefinitely)
                    target +=rconstants.movespindexer;
                    pendingMove = false;
                }

                // after 3 balls, move to next path state once follower done
                if ((ballCount >= 3||pathTimer.getElapsedTimeSeconds()>4.5) && !follower.isBusy()) {
                    target-=750;
                    setPathState(5);
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
                    transfermover.setPosition(rconstants.transfermoverscore);
                }
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>3&& pathTimer.getElapsedTimeSeconds()<4 &&flywheel.getVelocity()>=1150) {
                    target =8*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.3&&pathTimer.getElapsedTimeSeconds()<4.6) {
                    target =9*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.6&&pathTimer.getElapsedTimeSeconds()<4.8) {
                    target =10*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.8&&pathTimer.getElapsedTimeSeconds()<5.1){
                    transfermover.setPosition(rconstants.transfermoverfull);

                }
                if(pathTimer.getElapsedTimeSeconds()>5.1){
                    setPathState(7);
                }

                break;
            case 7:

                ballCount=0;
                setPathState(8);

                break;
            case 8:

                setPathState(9);

                break;
            case 9:
                //move to beginning of balls 4,5,6
                follower.followPath(Path4);

                transfermover.setPosition(rconstants.transfermoveridle);
                intake.setPower(1);

                setPathState(10);

                break;
            case 10:
                if(!follower.isBusy()) {
                    //picks up balls 4,5,6
                    transfer.setPower(0);
                    follower.setMaxPower(0.5);
                    follower.followPath(Path5);
                    setPathState(11);
                }
                break;
            case 11:

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
                if (pendingMove && distance.getDistance(DistanceUnit.CM)>4 && distance.getDistance(DistanceUnit.CM)<6) {
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
                    follower.followPath(Path6);
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
                    transfermover.setPosition(rconstants.transfermoverscore);
                }
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>3&& pathTimer.getElapsedTimeSeconds()<4 &&flywheel.getVelocity()>=1150) {
                    target =14*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.3&&pathTimer.getElapsedTimeSeconds()<4.6) {
                    target =15*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.6&&pathTimer.getElapsedTimeSeconds()<4.8) {
                    target =16*rconstants.movespindexer;
                }
                if(pathTimer.getElapsedTimeSeconds()>4.8&&pathTimer.getElapsedTimeSeconds()<5.1){
                    transfermover.setPosition(rconstants.transfermoverfull);

                }
                if(pathTimer.getElapsedTimeSeconds()>5.1){
                    setPathState(15);
                }
                break;
            case 15:
                follower.followPath(Path7);
                //shoot ball 6
                setPathState(-1);

                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds()>2) {
                    //go to beggining of balls 7,8,9
                    follower.followPath(Path7);
                    transfermover.setPosition(rconstants.transfermoveridle);
                    transfer.setPower(1);

                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    //pick up 7,8,9
                    follower.followPath(Path8);
                    setPathState(18);
                }
                break;
            case 18:
                //spindexer.setPower(0.5);
                if(!follower.isBusy())
                {
                    setPathState(-1);
                }


        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        colorSensor.getNormalizedColors();
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        turretL.setPower(0);
        turretR.setPower(0);
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
