package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name = "auton left blue", group = "Examples")
public class autonpathing extends OpMode {

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose p_r1 = new Pose(41.690140845070424, 35.605633802816904, Math.toRadians(180));
    private final Pose f_r1 = new Pose(12.619718309859156, 35.605633802816904, Math.toRadians(180));
    private final Pose deposit = new Pose(56.11267605633803, 8.563380281690137, Math.toRadians(90));
    private final Pose p_r2 = new Pose(43.267605633802816, 59.492957746478865, Math.toRadians(180));
    private final Pose f_r2 = new Pose(13.746478873239438, 59.492957746478865, Math.toRadians(180));
    private final Pose p_r3 = new Pose(41.690140845070424, 84.28169014084507, Math.toRadians(180));
    private final Pose f_r3 = new Pose(15.774647887323944, 83.83098591549296, Math.toRadians(180));

    public Path move_r1;
    public Path pickup_r1;
    public Path score_r1;
    public Path move_r2;
    public Path pickup_r2;
    public Path score_r2;
    public Path move_r3;
    public Path pickup_r3;

    private DcMotorEx spindexer;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private NormalizedColorSensor colorSensor3;
    private float[] hsvValues1 = new float[3];
    private float[] hsvValues2 = new float[3];
    private float[] hsvValues3 = new float[3];
    private boolean targetPurple = true;
    private boolean idle = false;
    private int[] classifier = new int[9];
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private String motif;
    private double transfermoverpos = 0.5;

    public void buildPaths() {
        move_r1 = new Path(new BezierLine(startPose, p_r1));
        move_r1.setLinearHeadingInterpolation(startPose.getHeading(), p_r1.getHeading());
        pickup_r1 = new Path(new BezierLine(p_r1, f_r1));
        pickup_r1.setLinearHeadingInterpolation(p_r1.getHeading(), f_r1.getHeading());
        score_r1 = new Path(new BezierLine(f_r1, deposit));
        score_r1.setLinearHeadingInterpolation(f_r1.getHeading(), deposit.getHeading());
        move_r2 = new Path(new BezierLine(deposit, p_r2));
        move_r2.setLinearHeadingInterpolation(deposit.getHeading(), p_r2.getHeading());
        pickup_r2 = new Path(new BezierLine(p_r2, f_r2));
        pickup_r2.setLinearHeadingInterpolation(p_r2.getHeading(), f_r2.getHeading());
        score_r2 = new Path(new BezierLine(f_r2, deposit));
        score_r2.setLinearHeadingInterpolation(f_r2.getHeading(), deposit.getHeading());
        move_r3 = new Path(new BezierLine(deposit, p_r3));
        move_r3.setLinearHeadingInterpolation(deposit.getHeading(), p_r3.getHeading());
        pickup_r3 = new Path(new BezierLine(p_r3, f_r3));
        pickup_r3.setLinearHeadingInterpolation(p_r3.getHeading(), f_r3.getHeading());
    }

    @Override
    public void init() {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "cs1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        colorSensor3 = hardwareMap.get(NormalizedColorSensor.class, "cs3");
        transfer = hardwareMap.get(CRServoImplEx.class, "transfer");
        transfermover = hardwareMap.get(ServoImplEx.class, "transfermover");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motif = "PGP";
    }

    @Override
    public void loop() {
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
