package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
@Config
@Autonomous(name = "red_faraway", group = "Examples")
public class farawayauton extends OpMode {
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
    public static int ball1_pos=950;
    public static int ball2_pos=950;
    public static int ball3_pos=950;





    private Timer pathTimer, actionTimer, opmodeTimer,goonTimer;
    private int pathState=0;
    private final Pose startPose = new Pose(87, 8, Math.toRadians(90));

    public PathChain firstpath;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public static int moveincrement = 2786/3;
    public static double constraint =1;
    /*
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
        */
    private double transfermoverpos = 0.5;

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(87.000, 8.000), new Pose(95.67523680649526, 33.12584573748308))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
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
        goonTimer=new Timer();
        opmodeTimer.resetTimer();
        transfer = hardwareMap.get(CRServoImplEx.class, "transfer");
        flywheel = hardwareMap.get(DcMotorEx.class,"shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfermover=hardwareMap.get(ServoImplEx.class,"transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //motif = "PGP";
        transfermover.setPosition(transfermoveridle);
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v,a,s)
                .build();


    }
    private void indexgoto(int target) {
        spindexer.setTargetPosition(target);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(0.5);
    }
    public void autonomousPathUpdate() {
        int pos = spindexer.getCurrentPosition();
        switch (pathState) {
            case 0:
                transfermover.setPosition(transfermoverscore);
                targetTicksPerSecond=1550;

                if(flywheel.getVelocity()>targetTicksPerSecond-25){
                    setPathState(1);
                }

                //shoot 2 balls
                break;
            case 1:
                transfer.setPower(1);


                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>3&& pathTimer.getElapsedTimeSeconds()<4 ) {
                    indexgoto(moveincrement*2 + pos);
                }
                if(pathTimer.getElapsedTimeSeconds()>6&&pathTimer.getElapsedTimeSeconds()<7&&flywheel.getVelocity()>targetTicksPerSecond-25) {
                    indexgoto(moveincrement +pos);
                    transfermover.setPosition(transfermoveridle);
                }
                if(pathTimer.getElapsedTimeSeconds()>7.8&&pathTimer.getElapsedTimeSeconds()<9.2&&flywheel.getVelocity()>targetTicksPerSecond-25){
                    transfermover.setPosition(transfermoverfull);
                }
                if(pathTimer.getElapsedTimeSeconds()>10){
                    setPathState(2);
                }

                //shoot third ball
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds()>1) {
                    //move to begening of 1,2,3
                    follower.followPath(Path1);
                    transfermover.setPosition(transfermoveridle);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
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
        autonomousPathUpdate();
        cs.setGoal(new KineticState(0,targetTicksPerSecond));
        KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
        flywheel.setPower(cs.calculate(current1));




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
