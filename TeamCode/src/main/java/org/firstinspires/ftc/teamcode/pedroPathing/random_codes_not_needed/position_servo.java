package org.firstinspires.ftc.teamcode.pedroPathing.random_codes_not_needed;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes.constants_testing;

import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@Disabled
@TeleOp(name="positionservo")
public class position_servo extends LinearOpMode {

    // ===================== DRIVE =====================
    private DcMotor lf, lb, rf, rb;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;

    // ===================== OTHER SUBSYSTEMS =====================
    private DcMotorEx flywheel, intake, spindexer;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private Servo hood;

    public static NormalizedColorSensor colorSensor;
    DistanceSensor distance;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p = 0.0039, i = 0, d = 0.0000005;
    public static double v = 0.000372, a = 0.7, s = 0.0000005;
    public static double p1=0.0084,i1=0,d1=
            0.000005;
    float[] hsv = new float[3];
    int ballCount = 0;
    public static int ballshot = 0;
    boolean colorPreviouslyDetected = false;

    // Slot tracking
    int[] ballSlots = new int[]{0, 0, 0}; // 0 empty, 1 purple, 2 green
    boolean sorting = false;
    int[] sortTarget = new int[]{0, 0, 0};

    public static double targetTicksPerSecond = 200;
    public static double shootclose = 1000;
    public static double shootfar = 1600;
    public static double shooteridle = 200;
    /*public static double turretp = 0.002;
    public static double turreti = 0;
    public static double turretd = 0.00000005;
    public static double turretv = 0.0000372;
    public static double turreta = 0.007;
    public static double turrets = 0.05;*/
    public static double ticksPerDegree = 126.42;
    public static double START_X = 35.285;
    public static double START_Y = 77.683;
    public static double START_HEADING_DEG = 133.5;
    public static double TARGET_X = 8;
    public static double TARGET_Y=136;
    // ===================== LIMELIGHT + TURRET FUSED AUTOALIGN =====================

    private Servo turretL;
    private CRServo turretR;

    // dedicated encoder you configured in RC as "turret_enc"
    private DcMotorEx turretEnc;

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;
    public static double targetTicks = 0;
    public static double ticks;
    public static double spindexerPIDspeed= 0.1;
    //private ControlSystem turretPID;
    private static final double FIELD_WIDTH = 144.0;     // inches
    private static final double FIELD_MID_X = FIELD_WIDTH / 2.0; // 72
    private static final double HEADING_MID_RAD = Math.toRadians(90.0); // 90 deg

    // X' = 72 + (72 - X) = 144 - X
    private static double blueToRedX(double x) {
        return FIELD_MID_X + (FIELD_MID_X - x);
    }
    // ---- your values ----
    public static String pp = "pp";

    public static double deadbandDeg = 2.0;
    public static double edgePauseSec = 0.1;

    // You provided 1.5; we clamp to [0..1] internally
    public static double errAlpha = 1.5;

    public static double forwardPodY = -5.46;
    public static double strafePodX = -1.693;
    public static DistanceUnit ppUnit = DistanceUnit.INCH;

    public static GoBildaPinpointDriver.GoBildaOdometryPods podType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
    public static GoBildaPinpointDriver.EncoderDirection forwardDir =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection strafeDir =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static double kP_hold = 0.02;
    public static double kP_track = 0.015;

    public static double maxHoldPower = 0.3;
    public static double maxTrackPower = 0.15;

    public static double maxTurretDeg = 60;
    public static double minTurretDeg = -60;

    public static int pipelineIndex = 2;
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
    public static double tyOffsetDeg = -3.5;
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
    private Timer currentTimer;
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
    public static Timer shottimer;
    public static boolean shots=true;

    // ===================== HELPERS =====================
    public double distancefromll(double ta) {
        return (71.7321 * (Math.pow(ta, -0.4550)));
    }
    public double velocityfromdistance(double distance){
        return 562.47005* Math.pow(distance,0.202468);
    }
    public static double hoodpos;

    @Override
    public void runOpMode() {
        shottimer= new Timer();
        currentTimer=new Timer();
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        constants_testing.initHardware(hardwareMap);
        // drive motors
        lf = constants_testing.lf;
        lb = constants_testing.lb;
        rf = constants_testing.rf;
        rb = constants_testing.rb;
        boolean shotLatched = false;

        elapsedtime = new ElapsedTime();

        // this just sets the bulk reading mode for each hub
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        elapsedtime.reset();
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        // turret
        //turretL = rconstants.turretL;
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretL.setPosition(0.5); // center on init
        turretR = constants_testing.turretR;

        // limelight


        // turret encoder (your new config name)
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pp);
        configurePinpoint(pinpoint);

        // shooter / intake / spindexer
        flywheel = constants_testing.flywheel;
        hood = constants_testing.hood;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = constants_testing.intake;

        spindexer = constants_testing.spindexer;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = constants_testing.transfer;
        transfermover = constants_testing.transfermover;
        transfermover.setPosition(constants_testing.transfermoveridle);

        colorSensor = constants_testing.colorSensor;
        colorSensor.setGain(constants_testing.csgain);
        distance = (DistanceSensor) colorSensor;

        cs1 = ControlSystem.builder()
                .posPid(p1, i1, d1)
                .build();
        cs = ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v, a, s)
                .build();

        int target = 0;

        // ===================== DRIVE THREAD (KEEP YOUR LOGIC) =====================
        Thread driveThread = new Thread(() -> {
            while (opModeIsActive()) {
                pinpoint.update();
                double fieldAngleDeg =
                        Math.toDegrees(
                                Math.atan2(
                                        TARGET_Y - pinpoint.getPosY(DistanceUnit.INCH),
                                        TARGET_X - pinpoint.getPosX(DistanceUnit.INCH)
                                )
                        ) - pinpoint.getHeading(AngleUnit.DEGREES);

                ticks = fieldAngleDeg * (ticksPerDegree/10.0);
                targetTicks = ticks;
                if(targetTicks > (60*ticksPerDegree/10.0)){
                    targetTicks=60*ticksPerDegree/10.0;
                } else if (targetTicks < (-60*ticksPerDegree/10.0)){
                    targetTicks=-60*ticksPerDegree/10.0;
                }

                KineticState current =
                        new KineticState(
                                turretEnc.getCurrentPosition()/10.0
                        );

                /*if (Math.abs(gamepad2.right_stick_x) < 0.05) {
                    turretPID = ControlSystem.builder()
                            .posPid(turretp, turreti, turretd)
                            .basicFF(turretv,turreta,turrets)
                            .build();

                    turretPID.setGoal(new KineticState(targetTicks));
                    turretL.setPower(-turretPID.calculate(current));
                } else {
                    turretL.setPower(gamepad2.right_stick_x);
                    targetTicks = turretEnc.getCurrentPosition()/10.0;
                }*/

                // REPLACE the entire turret block in driveThread with this:

                if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
                    // Manual override: update targetTicks FIRST, then compute position
                    targetTicks += gamepad2.right_stick_x * 2.0;
                    double maxTicks = 90.0 * ticksPerDegree / 10.0;
                    targetTicks = Math.max(-maxTicks, Math.min(maxTicks, targetTicks));
                }

// NOW compute servoPos from the already-clamped targetTicks
                double turretDeg = (targetTicks * 10.0) / ticksPerDegree;
                double servoPos = 0.5 - (turretDeg / 90.0) * 0.5;
                servoPos = Math.max(0.0, Math.min(1.0, servoPos));  // full ±90° = full [0,1]

                turretL.setPosition(servoPos);
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                lf.setPower((y + x + rx) / denom);
                lb.setPower((y - x + rx) / denom);
                rf.setPower((y - x - rx) / denom);
                rb.setPower((y + x - rx) / denom);

                if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper) {
                    transfermover.setPosition(constants_testing.transfermoverscore);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if (gamepad2.right_trigger > 0 && gamepad2.right_bumper) {
                    transfermover.setPosition(constants_testing.transfermoverfull);
                    transfer.setPower(gamepad2.right_trigger);
                }
                if (gamepad2.right_trigger == 0) {
                    transfermover.setPosition(constants_testing.transfermoveridle);
                    transfer.setPower(0);
                }

                if(intake.getCurrent(CurrentUnit.AMPS)<6.5) {
                    intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    currentTimer.resetTimer();
                } else if(currentTimer.getElapsedTimeSeconds()>1.2){
                    intake.setPower(-1);
                } else{
                    intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                }


                if(getDistance()>110){
                    farmode=true;
                } else{
                    farmode=false;
                }
                if(getDistance()>=60){
                    hood.setPosition(constants_testing.hoodtop);

                    if(getDistance()>60&&getDistance()<90){
                        targetTicksPerSecond = velocityfromdistance(getDistance())-40;

                    }else{
                        targetTicksPerSecond = velocityfromdistance(getDistance());
                    }
                } else{
                    if (gamepad2.y) {
                        targetTicksPerSecond = constants_testing.shootfar;
                        hood.setPosition(constants_testing.hoodtop);
                        farmode=true;
                    }
                    else if (gamepad2.b){
                        targetTicksPerSecond = constants_testing.shootclose;
                        hood.setPosition(constants_testing.hoodtop);
                        farmode=false;
                    }
                    else if (gamepad2.a) {
                        targetTicksPerSecond = constants_testing.shooteridle;
                        hood.setPosition(constants_testing.hoodbottom);
                        farmode=false;
                    } else{
                        hood.setPosition(constants_testing.hoodbottom);
                        targetTicksPerSecond=constants_testing.shooteridle;
                        farmode=false;
                    }
                }

                cs.setGoal(new KineticState(0,targetTicksPerSecond));
                KineticState current1 = new KineticState(flywheel.getCurrentPosition(),flywheel.getVelocity());
                flywheel.setPower(cs.calculate(current1));
            }
        });

        // ===================== TURRET THREAD (ALWAYS RUNS) =====================

        // IMPORTANT: init edge-search sentinel
        sweepTargetDeg = Double.NaN;

        waitForStart();

        driveThread.start();

        // ============================================================
        //                     MAIN LOOP
        // ============================================================
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            // after the first time, it won't actually send new commands
            telemetry.addData("Loop Times", elapsedtime.milliseconds());
            elapsedtime.reset();


            if(gamepad2.ps){
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                target=0;
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            boolean intakeRunning = Math.abs(intake.getPower()) > 0.05;

            if(spindexer.getCurrentPosition()%constants_testing.movespindexer <= 100 || spindexer.getCurrentPosition()%constants_testing.movespindexer >= constants_testing.movespindexer-100) {
                colorSensor.getNormalizedColors();
                Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
            }

            float hue = hsv[0];

            boolean isPurple = (hue >= 195 && hue <= 230);
            boolean isGreen = (hue >= 140 && hue <= 180);

            //distance
            boolean colorDetected = distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<6;

            if (intakeRunning && colorDetected && !colorPreviouslyDetected && ballCount < 3) {
                if(distance.getDistance(DistanceUnit.CM)>3 && distance.getDistance(DistanceUnit.CM)<7) {
                    //sleep(200);
                    target += constants_testing.movespindexer;

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

                    target += constants_testing.movespindexer;

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
            cs1 = ControlSystem.builder()
                    .posPid(p1, i1, d1)
                    .build();
            KineticState current2 = new KineticState(spindexer.getCurrentPosition());
            cs1.setGoal(new KineticState(target));

            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                spindexer.setPower(-spindexerPIDspeed*cs1.calculate(current2));
            } else if(farmode){
                spindexer.setPower(0.6*gamepad2.left_stick_y);
                target = spindexer.getCurrentPosition();
            } else{
                spindexer.setPower(0.6*gamepad2.left_stick_y);
                target = spindexer.getCurrentPosition();
            }

            // ---------- SHOOTER ----------

            if(gamepad2.left_bumper){
                int moveamount = constants_testing.movespindexer-(target%constants_testing.movespindexer);
                target+=moveamount;
                sleep(300);


            }

            while(gamepad1.left_trigger>0){
                if(ballCount==3&&!isGreen){
                    target+=constants_testing.movespindexer;
                    sleep(300);
                }
            }
            if(gamepad2.right_stick_button){
                pinpoint.setPosX(26.37,DistanceUnit.INCH);
                pinpoint.setPosY(131.69,DistanceUnit.INCH);
                pinpoint.setHeading(143,AngleUnit.DEGREES);
                //*TARGET_Y=125.15254237288136;
                //*TARGET_X=121.89830508474577;
            }
            // ---------- TELEMETRY ----------
            telemetry.addData("Hue", hue);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Slots", ballSlots[0] + "," + ballSlots[1] + "," + ballSlots[2]);
            telemetry.addData("Sorting", sorting);
            telemetry.addData("Sort Target", sortTarget[0] + "," + sortTarget[1] + "," + sortTarget[2]);
            telemetry.addData("Target", target);
            telemetry.addData("Spindexer calculated speed:", -0.25*cs1.calculate(current2));
            telemetry.addData("Spindexer current power: ", spindexer.getPower());
            telemetry.addData("spindexer_pos", spindexer.getCurrentPosition());
            telemetry.addData("Flywheel Target: ", targetTicksPerSecond);
            telemetry.addData("Flywheel Velocity: ", flywheel.getVelocity());
            telemetry.addData("distance", getDistance());
            telemetry.addData("distance of spindexer", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("bot_x", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("bot_y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("headung", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.update();


        }

    }
    private double getDistance(){
        double dy = TARGET_Y-pinpoint.getPosY(DistanceUnit.INCH);
        double dx= TARGET_X-pinpoint.getPosX(DistanceUnit.INCH);
        double distance = Math.sqrt(Math.pow(dy,2)+Math.pow(dx,2));
        return distance;
    }

    // ===================== Pinpoint setup =====================
    private void configurePinpoint(GoBildaPinpointDriver pp) {

        pp.setOffsets(-5.46, -1.693, DistanceUnit.INCH);

        pp.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pp.resetPosAndIMU();

        pp.setPosition(new Pose2D(
                DistanceUnit.INCH,
                START_X,
                START_Y,
                AngleUnit.DEGREES,
                START_HEADING_DEG
        ));
    }
    // ===================== Turret power helper =====================
    //private void setTurretPower(double pwr) {
        //turretL.setPower(pwr);
        //turretR.setPower(pwr);
    //}

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