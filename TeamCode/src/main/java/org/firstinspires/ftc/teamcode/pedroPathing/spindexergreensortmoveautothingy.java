package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp
@Disabled
public class spindexergreensortmoveautothingy extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
    private DcMotorEx flywheel, intake, spindexer;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private CRServo turretL;
    private CRServo turretR;
    private double targetx;

    public static NormalizedColorSensor colorSensor;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p=0.0039,i=0,d=0.0000005;
    public static double v=0.000372,a=0.7,s=0.0000005;
    public static double p1=0.0009,i1=0,d1=0;
    public static double llturretspeed = 0.2;

    float[] hsv = new float[3];

    int ballCount = 0;
    boolean colorPreviouslyDetected = false;

    // Slot tracking
    int[] ballSlots = new int[]{0,0,0}; // 0 empty, 1 purple, 2 green
    boolean sorting = false;
    int[] sortTarget = new int[]{0,0,0};

    public static double targetTicksPerSecond=200;
    public static double shootclose = 1000;
    public static double shootfar=1600;
    public static double shooteridle = 200;
    public static boolean autoalign = false;
    private Limelight3A limelight;
    DistanceSensor distance;
    private boolean movedoffsetspindexer;
    private Servo hood;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
        int turretOscillationDirection = 0; // 0=left, 1=righ

        rconstants.initHardware(hardwareMap);

        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        turretL=rconstants.turretL;
        turretR=rconstants.turretR;
        limelight=rconstants.limelight;
        limelight.pipelineSwitch(1);
        flywheel = rconstants.flywheel;
        hood=rconstants.hood;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = rconstants.intake;

        spindexer = rconstants.spindexer;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = rconstants.transfer;
        transfermover = rconstants.transfermover;
        transfermover.setPosition(rconstants.transfermoveridle);

        colorSensor = rconstants.colorSensor;
        colorSensor.setGain(2.7f);

        cs1 = ControlSystem.builder()
                .posPid(p1,i1,d1)
                .build();
        cs = ControlSystem.builder()
                .velPid(p,i,d)
                .basicFF(v,a,s)
                .build();
        limelight.start();
        distance = (DistanceSensor) colorSensor;

        int target = 0;

        cs1 = ControlSystem.builder()
                .posPid(0.0009, 0, 0)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.05;

            // ---------- READ COLOR ----------
            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);

            float hue = hsv[0];
            boolean isPurple = (hue > 200 && hue < 300);
            boolean isGreen  = (hue > 95  && hue < 200);

            boolean colorDetected = (isPurple || isGreen);

            // ---------- BALL DETECTION ----------
            if (intakeRunning && colorDetected && !colorPreviouslyDetected && ballCount < 3) {
                if(distance.getDistance(DistanceUnit.CM)>4.5 && distance.getDistance(DistanceUnit.CM)<6) {
                    //sleep(200);
                    target += rconstants.movespindexer;

                    if (isPurple) ballSlots[ballCount] = 1;
                    if (isGreen) ballSlots[ballCount] = 2;

                    ballCount++;
                    colorPreviouslyDetected = true;
                }
            }

            if (!colorDetected) {
                colorPreviouslyDetected = false;
            }
            while(gamepad1.left_trigger>0){
                if(ballCount==3&&!isGreen){
                    target+=rconstants.movespindexer;
                    sleep(300);
                }
            }
            intake.setPower(gamepad2.right_trigger);
        }
    }


}
