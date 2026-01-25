package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp
public class multithreadteleop extends LinearOpMode {

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


    public double distancefromll(double ta)
    {

        double dis = (71.7321*(Math.pow(ta,-0.4550)));
        return dis;
    }
    public double targetvolfromll(double dis)
    {
        double vol = (7.1572*(Math.pow(dis,2.5726)));
        return vol;
    }
    @Override
    public void runOpMode(){


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
        colorSensor.setGain(rconstants.csgain);

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

        // ---------------------- DRIVE THREAD ----------------------
        Thread g1 = new Thread(() -> {
            while (opModeIsActive()) {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
                lf.setPower((y+x+rx)/denom);
                lb.setPower((y-x+rx)/denom);
                rf.setPower((y-x-rx)/denom);
                rb.setPower((y+x-rx)/denom);

                if(gamepad2.right_trigger>0 && !gamepad2.right_bumper){
                    transfermover.setPosition(rconstants.transfermoverscore);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if(gamepad2.right_trigger>0 && gamepad2.right_bumper){
                    transfermover.setPosition(rconstants.transfermoverfull);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if(gamepad2.right_trigger==0){
                    transfermover.setPosition(rconstants.transfermoveridle);
                    transfer.setPower(0);
                }
                intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }
        });

        waitForStart();
        g1.start();

        // ============================================================
        //                     MAIN LOOP
        // ============================================================
        while (opModeIsActive()) {
            LLResult llResult = limelight.getLatestResult();



            if(gamepad2.ps){
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                target=0;
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad2.back){
                autoalign=!autoalign;
                sleep(300);
            }
            if(ballCount==3&&!movedoffsetspindexer){
                sleep(100);
                target+=movespindexer/2;
                target-=750;
                movedoffsetspindexer=true;
            }
            if(gamepad2.left_trigger >0.1){
                target+=movespindexer/2;
            }
            if(autoalign) {
                if (Math.abs(gamepad2.left_stick_x) == 0) {
                    if (llResult != null) {
                        targetx = llResult.getTy();
                        telemetry.addData("targetx", llResult.getTy());

                        if (targetx >= 2.5) {
                            // not necesary but makes it move exactly one degree
                            turretL.setPower(llturretspeed);
                            turretR.setPower(llturretspeed);
                            turretOscillationDirection = 0;
                            //switch to negative and make other postive if goes wrong direction
                        } else if (targetx <= -2.5) {
                            turretL.setPower(-llturretspeed);
                            turretR.setPower(-llturretspeed);
                            turretOscillationDirection = 1;
                        } else if (targetx >= -5.5 && targetx <= 5.5) {
                            turretR.setPower(0);
                            turretL.setPower(0);
                        }
                    } else {
                        if (turretOscillationDirection == 0) {
                            turretL.setPower(-0.1);
                            turretR.setPower(-0.1);
                            sleep(500);
                            turretOscillationDirection = 1;
                        } else {
                            turretL.setPower(0.1);
                            turretR.setPower(0.1);
                            sleep(500);
                            turretOscillationDirection = 0;
                        }
                    }
                } else{
                    turretL.setPower(gamepad2.right_stick_x);
                    turretR.setPower(gamepad2.right_stick_x);
                }
            } else{
                turretL.setPower(gamepad2.right_stick_x);
                turretR.setPower(gamepad2.right_stick_x);
            }
            // ---------- INTAKE ----------
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.05;

            // ---------- READ COLOR ----------
            if(spindexer.getCurrentPosition()%rconstants.movespindexer <= 100 || spindexer.getCurrentPosition()%rconstants.movespindexer >= rconstants.movespindexer-100) {
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
            }

            float hue = hsv[0];

            boolean isPurple = (hue > 195 && hue < 220);
            boolean isGreen = (hue > 150 && hue < 170);

            boolean colorDetected = (isPurple || isGreen);

            // ---------- BALL DETECTION ----------
            if (intakeRunning && colorDetected && !colorPreviouslyDetected && ballCount < 3) {
                if(distance.getDistance(DistanceUnit.CM)>4.5 && distance.getDistance(DistanceUnit.CM)<7) {
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

            // ---------- SHOOTER ----------
            if (gamepad2.y) {
                targetTicksPerSecond = rconstants.shootfar;
                hood.setPosition(rconstants.hoodtop);
            }
            if (gamepad2.b){
                targetTicksPerSecond = rconstants.shootclose;
                hood.setPosition(rconstants.hoodbottom);
            }
            if (gamepad2.a) {
                targetTicksPerSecond = rconstants.shooteridle;
                hood.setPosition(rconstants.hoodbottom);
            }

            // Reset only ballCount (not slots)
            if (gamepad2.x) {
                ballCount = 0;
                movedoffsetspindexer=false;
            }

            // ---------- SORT BUTTONS ----------
            if (gamepad2.dpad_right && ballCount == 3 && !sorting) {
                //PPG
                sortTarget = new int[]{1,2,1};
                sorting = true;
            }
            if (gamepad2.dpad_up && ballCount == 3 && !sorting) {
                //PGP
                sortTarget = new int[]{2,1,1};
                sorting = true;
            }
            if (gamepad2.dpad_left && ballCount == 3 && !sorting) {
                //GPP
                sortTarget = new int[]{1,1,2};
                sorting = true;
            }

            // ---------- FORWARD-ONLY SORTING ----------
            if (sorting) {

                if (!(ballSlots[0] == sortTarget[0] &&ballSlots[1]==sortTarget[1]&& ballSlots[2]==sortTarget[2])) {

                    target += rconstants.movespindexer;

                    int temp = ballSlots[0];
                    ballSlots[0] = ballSlots[1];
                    ballSlots[1] = ballSlots[2];
                    ballSlots[2] = temp;

                    sleep(250);

                } else {
                    sorting = false;
                }
            }

            // ---------- SPINDEXER PID ----------
            KineticState current2 = new KineticState(spindexer.getCurrentPosition(), spindexer.getVelocity());
            cs1.setGoal(new KineticState(target));

            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                spindexer.setPower(-cs1.calculate(current2));
            } else {
                spindexer.setPower(gamepad2.left_stick_y);
                target = spindexer.getCurrentPosition();
            }

            // ---------- SHOOTER ----------
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            KineticState current1 = new KineticState(flywheel.getCurrentPosition(),flywheel.getVelocity());
            flywheel.setPower(cs.calculate(current1));
            if(gamepad2.left_bumper){
                int moveamount = rconstants.movespindexer-(target%rconstants.movespindexer);
                target+=moveamount;
                sleep(300);


            }

            while(gamepad1.left_trigger>0){
                if(ballCount==3&&!isGreen){
                    target+=rconstants.movespindexer;
                    sleep(300);
                }
            }



            // ---------- TELEMETRY ----------
            telemetry.addData("Hue", hue);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Slots", ballSlots[0] + "," + ballSlots[1] + "," + ballSlots[2]);
            telemetry.addData("Sorting", sorting);
            telemetry.addData("Sort Target", sortTarget[0] + "," + sortTarget[1] + "," + sortTarget[2]);
            telemetry.addData("Target", target);
            telemetry.addData("Autoalign:", autoalign);
            telemetry.addData("spindexer_pos", spindexer.getCurrentPosition());
            telemetry.addData("distance", distancefromll(llResult.getTa()));
            telemetry.addData("distance of spindexer", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
