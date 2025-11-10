package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

import com.bylazar.telemetry.PanelsTelemetry;
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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Autonomous(name = "auton_blue_big_triiangel", group = "Examples")
public class closebyauton extends OpMode {
    private Follower follower;
    public ServoImplEx transfermover;
    private DcMotorEx spindexer;
    private CRServoImplEx transfer;
    private DcMotorEx flywheel;
    private DcMotorEx intake;
    private DcMotorEx rb;
    private ControlSystem cs;
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
    public static double hoodtop = 0;
    public static double hoodbottom = 0.1;
    public static int ball1_pos=0;
    public static int ball2_pos=900;
    public static int ball3_pos=1800;

    public static int spindexer_target_pos=ball1_pos;




    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose startPose = new Pose(72, 8, Math.toRadians(140));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;
    private float[] hsvValues1 = new float[3];
    private float[] hsvValues2 = new float[3];
    private float[] hsvValues3 = new float[3];
    private boolean targetPurple = true;
    private boolean idle = false;
    private int[] classifier = new int[9];
    private String motif;
    private double transfermoverpos = 0.5;

    public void buildPaths() {
        Path1 = follower
            .pathBuilder()
            .addPath(
                    new BezierLine(new Pose(27.463, 131.821), new Pose(44.657, 86.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
            .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.657, 86.000), new Pose(21.700, 86.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.700, 86.000), new Pose(57.600, 86.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.600, 86.000), new Pose(42.746, 61.134))
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(42.746, 61.134), new Pose(22.000, 61.134))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.000, 61.134), new Pose(57.600, 86.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(57.600, 86.000), new Pose(39.403, 38.448))
                )
                .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(39.403, 38.448), new Pose(22.000, 38.687))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

    }

    @Override
    public void init() {
        //colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        //colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        //colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "cs3");
        transfer = hardwareMap.get(CRServoImplEx.class, "transfer");
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfermover=hardwareMap.get(ServoImplEx.class,"transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motif = "PGP";
        transfermover.setPosition(transfermoveridle);
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v,a,s)
                .build();


    }
    private void indexgoto(int target) {
        if (spindexer.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(0.8);
        }
        spindexer.setTargetPosition(target);
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                targetTicksPerSecond=1250;
                indexgoto(ball1_pos);
                if (pathTimer.getElapsedTimeSeconds() > 5) {
                    transfermover.setPosition(transfermoverscore);
                    transfer.setPower(1);
                    indexgoto(ball2_pos);
                    //shoot 2 balls
                    setPathState(1);
                }
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds()>2)
                {
                    transfermover.setPosition(transfermoverfull);
                    indexgoto(ball3_pos);
                    setPathState(2);
                    //shoot third ball
                }
            case 2:
                if(pathTimer.getElapsedTimeSeconds()>5)
                    //move to begening of 1,2,3
                     follower.followPath(Path1);
                transfermover.setPosition(transfermoveridle);
                transfer.setPower(0);
                intake.setPower(1);
                setPathState(3);

                break;
            case 3:
                if(!follower.isBusy()) {
                    //pick up 1,2,3
                    follower.followPath(Path2);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds()>1)
                    indexgoto(ball2_pos);
                if(pathTimer.getElapsedTimeSeconds()>2)
                    indexgoto(ball1_pos);
                setPathState(5);
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(Path3);
                    setPathState(6);
                    //move to shoot position
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    //shoot balls 1,2
                    transfer.setPower(1);
                    transfermover.setPosition(transfermoverscore);
                    //spidexer already on pos 1
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds()>0.5) {
                    indexgoto(ball2_pos);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds()>1)
                    //shoot ball 3
                    transfermover.setPosition(transfermoverfull);
                    indexgoto(ball3_pos);
                    setPathState(9);
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds()>2) {
                    //move to beginning of balls 4,5,6
                    follower.followPath(Path4);

                    transfermover.setPosition(transfermoveridle);
                    transfer.setPower(0);

                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    //picks up balls 4,5,6
                    follower.followPath(Path5);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds()>1)
                    indexgoto(ball2_pos);
                if(pathTimer.getElapsedTimeSeconds()>2)
                    indexgoto(ball1_pos);
                setPathState(12);
                break;
            case 12:
                if(!follower.isBusy()) {
                    //move to shooting position for balls 4,5,6
                    follower.followPath(Path6);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    //shoot balls 4
                    transfermover.setPosition(transfermoverscore);
                    transfer.setPower(1);

                    setPathState(14);
                }
                break;
            case 14:
                if(pathTimer.getElapsedTimeSeconds()>0.5)
                {
                    //shoot ball 5
                    indexgoto(2);
                    setPathState(15);
                }
                break;
            case 15:
                if(pathTimer.getElapsedTimeSeconds()>1)
                {
                    //shoot ball 6
                    transfermover.setPosition(transfermoverfull);
                    indexgoto(ball3_pos);
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTimeSeconds()>5) {
                    //go to beggining of balls 7,8,9
                    follower.followPath(Path7);
                    transfermover.setPosition(transfermoveridle);
                    transfer.setPower(0);

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
                if(pathTimer.getElapsedTimeSeconds()>4)
                {
                    if(pathTimer.getElapsedTimeSeconds()>1)
                        indexgoto(ball2_pos);
                    if(pathTimer.getElapsedTimeSeconds()>2)
                        indexgoto(ball1_pos);
                    if(!follower.isBusy())
                    {
                        setPathState(-1);
                    }
                }

        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void loop() {

        cs.setGoal(new KineticState(0,targetTicksPerSecond));
        KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
        flywheel.setPower(cs.calculate(current1));
        spindexer.setTargetPosition(spindexer_target_pos);






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
    }
}
