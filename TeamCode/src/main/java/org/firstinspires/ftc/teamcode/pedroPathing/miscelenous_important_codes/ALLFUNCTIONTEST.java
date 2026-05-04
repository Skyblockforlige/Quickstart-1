package org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Autonomous
public class ALLFUNCTIONTEST extends LinearOpMode {
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


    public static double auton_x =0;
    public static double auton_y =0;

    public static double auton_heading =0;

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
    @Override
    public void runOpMode(){
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
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
        transfermover.setPosition(rconstants.transfermoveridle);
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v,a,s)
                .build();
        cs1 = ControlSystem.builder()
                .posPid(p1)
                .build();
        hood.setPosition(constants_testing.hoodbottom);
        waitForStart();
        if(opModeIsActive()){
            actionTimer.resetTimer();
            hood.setPosition(constants_testing.hoodtop);
            intake.setPower(1);
            sleep(3000);
            intake.setPower(0);
            transfer.setPower(1);
            transfermover.setPosition(rconstants.transfermoverscore);
            sleep(3000);
            transfermover.setPosition(rconstants.transfermoveridle);
            transfer.setPower(0);

        }
        while(opModeIsActive()){
                if (flywheel.getVelocity() < (targetTicksPerSecond - 90)) {
                    flywheel.setPower(1);
                } else {
                    cs.setGoal(new KineticState(0, 200));
                    KineticState current1 = new KineticState(flywheel.getCurrentPosition(), flywheel.getVelocity());
                    flywheel.setPower(cs.calculate(current1));
                }

            turretL.setPosition(0.75);
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);



            KineticState current2 = new KineticState(spindexer.getCurrentPosition(),spindexer.getVelocity());
            cs1.setGoal(new KineticState(6*rconstants.movespindexer));
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
            telemetry.update();

        }
    }
}
