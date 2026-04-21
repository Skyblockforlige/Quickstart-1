package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp(name="testing")
public class position_servo_code extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;

    private DcMotorEx flywheel, intake, spindexer;
    private double shooter_reverse = 1;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private Servo hood;

    public volatile Pose llPedro = new Pose(0, 0, 0);
    public volatile boolean llValid = false;

    public static volatile double spindexer_speed_shooting = 0.6;

    private Limelight3A limelight;

    public static NormalizedColorSensor colorSensor;
    DistanceSensor distance;

    ControlSystem cs, cs1;

    public static int movespindexer = 2731;
    public static double p = 0.0039, i = 0, d = 0.0000005;
    public static double v = 0.000372, a = 0.7, s = 0.0000005;
    public static double p1 = 0.0084, i1 = 0, d1 = 0.000005;
    float[] hsv = new float[3];
    int ballCount = 0;
    public static int ballshot = 0;
    boolean colorPreviouslyDetected = false;

    int[] ballSlots = new int[]{0, 0, 0};
    boolean sorting = false;
    int[] sortTarget = new int[]{0, 0, 0};

    public static double targetTicksPerSecond = 200;
    public static double shootclose = 1000;
    public static double shootfar = 1600;
    public static double shooteridle = 200;
    public static double ticksPerDegree = 126.42;
    public static double START_X = 35.285;
    public static double START_Y = 77.683;
    public static double START_HEADING_DEG = 133.5;
    public static double TARGET_X = 12;
    public static double TARGET_Y = 136;

    private Servo turretL;
    private CRServo turretR;
    private DcMotorEx turretEnc;

    private GoBildaPinpointDriver pinpoint;
    public static double targetTicks = 0;
    public static double ticks;
    public static double spindexerPIDspeed = 0.1;
    public static double spindexer_speed_close = 0.1;
    public static double spindexer_speed_far = 0.05;

    public static String pp = "pp";
    public static int pipelineIndex = 5;

    private boolean farmode = false;
    private boolean movedoffsetspindexer;
    private Timer currentTimer;

    public static Timer shottimer;
    public static boolean shots = true;

    private static final double METERS_TO_INCHES = 39.3701;
    private static final double FIELD_SIZE_INCHES = 141.6;
    public static double Y_OFFSET_INCHES = 117.0;
    public static double x_OFFSET_INCHES = 3.5;
    public static double HEADING_OFFSET_DEG = 0.0;


    private static final double FIELD_HALF_INCHES = 72.0;
    public static double X_OFFSET_INCHES = 3.5;
    public static  double LL_CORRECTION_THRESHOLD_INCHES = 2.0;


    private Pose limelightMT2ToPedroPose(Pose3D llPose, double imuDeg) {
        double xIn = llPose.getPosition().x * METERS_TO_INCHES;
        double yIn = llPose.getPosition().y * METERS_TO_INCHES;
        double pedroX = xIn + (FIELD_HALF_INCHES * 2.0 / 2.0) + x_OFFSET_INCHES;
        double pedroY = yIn + FIELD_HALF_INCHES + Y_OFFSET_INCHES;
        double heading = normalizeAngleDeg(imuDeg + HEADING_OFFSET_DEG);
        return new Pose(pedroX, pedroY, Math.toRadians(heading));
    }


    private double normalizeAngleDeg(double deg) {
        deg = deg % 360.0;
        if (deg > 180.0)   deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }

    public double distancefromll(double ta) {
        return (71.7321 * (Math.pow(ta, -0.4550)));
    }

    public double velocityfromdistance(double distance) {
        //return (((0.0000121506 * distance - 0.00507176) * distance + 0.775497) * distance - 45.56069) * distance + 2023.94766;
        return 515.47005* Math.pow(distance,0.202468);
    }

    @Override
    public void runOpMode() {
        shottimer = new Timer();
        currentTimer = new Timer();
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        constants_testing.initHardware(hardwareMap);
        lf = constants_testing.lf;
        lb = constants_testing.lb;
        rf = constants_testing.rf;
        rb = constants_testing.rb;

        elapsedtime = new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        turretL = hardwareMap.get(Servo.class, "turretL");
        turretL.setPosition(0.5);
        turretR = constants_testing.turretR;

        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pp);
        configurePinpoint(pinpoint);

        flywheel      = constants_testing.flywheel;
        hood          = constants_testing.hood;
        intake        = constants_testing.intake;
        spindexer     = constants_testing.spindexer;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer      = constants_testing.transfer;
        transfermover = constants_testing.transfermover;
        colorSensor   = constants_testing.colorSensor;
        distance      = (DistanceSensor) colorSensor;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();

        int target = 0;

        Thread driveThread = new Thread(() -> {
            while (opModeIsActive()) {
                pinpoint.update();

                double currentHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
                limelight.updateRobotOrientation(currentHeadingDeg);

                LLResult latest = limelight.getLatestResult();


// inside the drive thread, replace the llValid block with this:
                if (latest != null && latest.isValid()) {
                    Pose3D mt2Pose = latest.getBotpose_MT2();
                    if (mt2Pose != null) {
                        llPedro = limelightMT2ToPedroPose(mt2Pose, currentHeadingDeg);
                        llValid = true;

                        double driftX = Math.abs(llPedro.getX() - pinpoint.getPosX(DistanceUnit.INCH));
                        double driftY = Math.abs(llPedro.getY() - pinpoint.getPosY(DistanceUnit.INCH));

                        if (driftX > LL_CORRECTION_THRESHOLD_INCHES) {
                            pinpoint.setPosX(llPedro.getX(), DistanceUnit.INCH);
                        }
                        if (driftY > LL_CORRECTION_THRESHOLD_INCHES) {
                            pinpoint.setPosY(llPedro.getY(), DistanceUnit.INCH);
                        }
                    }
                } else {
                    llValid = false;
                }

                double fieldAngleDeg = Math.toDegrees(Math.atan2(
                        TARGET_Y - pinpoint.getPosY(DistanceUnit.INCH),
                        TARGET_X - pinpoint.getPosX(DistanceUnit.INCH)
                )) - pinpoint.getHeading(AngleUnit.DEGREES);

                ticks = fieldAngleDeg * (ticksPerDegree / 10.0);

                if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
                    targetTicks += gamepad2.right_stick_x * 2.0;
                } else {
                    targetTicks = ticks;
                }

                double maxTicks = 90.0 * ticksPerDegree / 10.0;
                targetTicks = Math.max(-maxTicks, Math.min(maxTicks, targetTicks));

                double turretDeg = (targetTicks * 10.0) / ticksPerDegree;
                double servoPos  = 0.5 - (turretDeg / 90.0) * 0.5;
                turretL.setPosition(Math.max(0.0, Math.min(1.0, servoPos)));

                double y    = -gamepad1.left_stick_y;
                double x    =  gamepad1.left_stick_x * 1.1;
                double rx   =  gamepad1.right_stick_x;
                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                lf.setPower((y + x + rx) / denom);
                lb.setPower((y - x + rx) / denom);
                rf.setPower((y - x - rx) / denom);
                rb.setPower((y + x - rx) / denom);

                if (gamepad2.right_trigger > 0) {
                    transfermover.setPosition(gamepad2.right_bumper
                            ? constants_testing.transfermoverfull
                            : constants_testing.transfermoverscore);
                    transfer.setPower(gamepad2.right_trigger);
                } else {
                    transfermover.setPosition(constants_testing.transfermoveridle);
                    transfer.setPower(0);
                }

                if (intake.getCurrent(CurrentUnit.AMPS) < 6.5) {
                    intake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                    currentTimer.resetTimer();
                } else if (currentTimer.getElapsedTimeSeconds() > 1.2) {
                    intake.setPower(-1);
                }

                double dist = getDistance();
                if (dist >= 50) {
                    hood.setPosition(constants_testing.hoodtop);
                    targetTicksPerSecond = velocityfromdistance(dist);
                } else {
                    if      (gamepad2.y) { targetTicksPerSecond = constants_testing.shootfar;   hood.setPosition(constants_testing.hoodtop);    }
                    else if (gamepad2.b) { targetTicksPerSecond = constants_testing.shootclose;  hood.setPosition(constants_testing.hoodtop);    }
                    else if (gamepad2.a) { targetTicksPerSecond = constants_testing.shooteridle; hood.setPosition(constants_testing.hoodbottom); }
                    else                 { hood.setPosition(constants_testing.hoodbottom); targetTicksPerSecond = constants_testing.shooteridle; }
                }

                cs = ControlSystem.builder().velPid(p, i, d).basicFF(v, a, s).build();
                cs.setGoal(new KineticState(0, targetTicksPerSecond));
                flywheel.setPower(cs.calculate(new KineticState(
                        flywheel.getCurrentPosition(), flywheel.getVelocity())));
            }
        });

        waitForStart();
        driveThread.start();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) { hub.clearBulkCache(); }
            elapsedtime.reset();

            if (gamepad2.ps) {
                spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                target = 0;
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (ballCount == 3 && !movedoffsetspindexer) {
                target += (movespindexer / 2) - 750;
                movedoffsetspindexer = true;
            }
            if (gamepad2.left_trigger > 0.1) target += movespindexer / 2;

            colorSensor.getNormalizedColors();
            Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
            boolean colorDetected = distance.getDistance(DistanceUnit.CM) > 3
                    && distance.getDistance(DistanceUnit.CM) < 6;

            if (Math.abs(intake.getPower()) > 0.05 && colorDetected
                    && !colorPreviouslyDetected && ballCount < 3) {
                target += constants_testing.movespindexer;
                if (hsv[0] >= 195 && hsv[0] <= 230) ballSlots[ballCount] = 1;
                if (hsv[0] >= 140 && hsv[0] <= 180) ballSlots[ballCount] = 2;
                ballCount++;
                colorPreviouslyDetected = true;
            }
            if (!colorDetected) colorPreviouslyDetected = false;

            if (gamepad2.x) { ballCount = 0; movedoffsetspindexer = false; }

            if (gamepad2.dpad_right && ballCount == 3 && !sorting) {
                sortTarget = new int[]{1, 2, 1};
                sorting = true;
            }
            if (sorting) {
                if (!(ballSlots[0] == sortTarget[0]
                        && ballSlots[1] == sortTarget[1]
                        && ballSlots[2] == sortTarget[2])) {
                    target += constants_testing.movespindexer;
                    int temp     = ballSlots[0];
                    ballSlots[0] = ballSlots[1];
                    ballSlots[1] = ballSlots[2];
                    ballSlots[2] = temp;
                    sleep(250);
                } else {
                    sorting = false;
                }
            }
            if (getDistance()>=110)
                spindexer_speed_shooting=0.4;
            else
                spindexer_speed_shooting=0.6;



            cs1 = ControlSystem.builder().posPid(p1, i1, d1).build();
            cs1.setGoal(new KineticState(target));
            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                spindexer.setPower(-spindexerPIDspeed
                        * cs1.calculate(new KineticState(spindexer.getCurrentPosition())));
            } else {
                if(gamepad2.left_stick_y<0)
                    shooter_reverse = -1;
                else
                    shooter_reverse = 1;
                spindexer.setPower(spindexer_speed_shooting * shooter_reverse);
                target = spindexer.getCurrentPosition();
            }

            if (gamepad2.left_bumper) {
                target += constants_testing.movespindexer
                        - (target % constants_testing.movespindexer);
                sleep(300);
            }

            if (gamepad2.right_stick_button) {
                pinpoint.setPosX(26.37, DistanceUnit.INCH);
                pinpoint.setPosY(131.69, DistanceUnit.INCH);
                pinpoint.setHeading(143, AngleUnit.DEGREES);
            }

            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("Loop times", elapsedtime.toString());
            telemetry.addData("Target",     target);
            telemetry.addData("Distance (in)", String.format("%.2f in", getDistance()));
            telemetry.addData("shooter vel", targetTicksPerSecond);
            telemetry.addData("Pinpoint X (in)",        String.format("%.3f in", pinpoint.getPosX(DistanceUnit.INCH)));
            telemetry.addData("Pinpoint Y (in)",        String.format("%.3f in", pinpoint.getPosY(DistanceUnit.INCH)));
            telemetry.addData("Pinpoint Heading (deg)", String.format("%.2f°",   pinpoint.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("LL Valid", llValid);
            if (llValid) {
                telemetry.addData("LL X (in)",        String.format("%.3f in", llPedro.getX()));
                telemetry.addData("LL Y (in)",        String.format("%.3f in", llPedro.getY()));
                telemetry.addData("LL Heading (deg)", String.format("%.2f°", Math.toDegrees(llPedro.getHeading())));
            }
            telemetry.addData("Intake Current Amps: ", intake.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }

    private double getDistance() {
        return Math.sqrt(
                Math.pow(TARGET_Y - pinpoint.getPosY(DistanceUnit.INCH), 2)
                        + Math.pow(TARGET_X - pinpoint.getPosX(DistanceUnit.INCH), 2));
    }
    public double getSpindexerSpeed(double dist) {
        return 0.05;
    }

    private void configurePinpoint(GoBildaPinpointDriver pp) {
        pp.setOffsets(-5.46, -1.693, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pp.resetPosAndIMU();
        pp.setPosition(new Pose2D(
                DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING_DEG));
    }
}