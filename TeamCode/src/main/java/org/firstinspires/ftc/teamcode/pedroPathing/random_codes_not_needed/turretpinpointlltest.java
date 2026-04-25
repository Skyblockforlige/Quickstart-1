package org.firstinspires.ftc.teamcode.pedroPathing.random_codes_not_needed;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes.rconstants;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@Disabled
@TeleOp
public class turretpinpointlltest extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
    private Timer currentTimer;

    private DcMotorEx flywheel, intake, spindexer;
    private CRServoImplEx transfer;
    private ServoImplEx transfermover;
    private CRServo turretServo;
    private DcMotorEx turretEnc;

    private GoBildaPinpointDriver pinpoint;
    private ControlSystem turretPID;

    private Limelight3A limelight;
    private DistanceSensor distance;

    public static double START_X = 35.285;
    public static double START_Y = 77.683;
    public static double START_HEADING_DEG = 133.5;
    public static double TARGET_X = 144;
    public static double TARGET_Y=135;
    private static final double FIELD_WIDTH = 144.0;     // inches
    private static final double FIELD_MID_X = FIELD_WIDTH / 2.0; // 72
    private static final double HEADING_MID_RAD = Math.toRadians(90.0); // 90 deg

    // X' = 72 + (72 - X) = 144 - X
    private static double blueToRedX(double x) {
        return FIELD_MID_X + (FIELD_MID_X - x);
    }

    public static double p2 = 0.00035;
    public static double i2 = 0.0000000005;
    public static double d2 = 0.0000000002;
    public static double turretp = 0.002;
    public static double turreti = 0;
    public static double turretd = 0.00000005;
    public static double turretv = 0.0000372;
    public static double turreta = 0.007;
    public static double turrets = 0.05;
    public static double ticksPerDegree = 126.42;

    public static double llAngleScale = 1.0;
    public static boolean autoalign = false;

    public static double p = 0.0039, i = 0, d = 0.0000005;
    public static double v = 0.000372, a = 0.7, s = 0.0000005;
    public static double p1=0.0084,i1=0,d1=0.000005;
    public static double targetTicksPerSecond=200;


    ControlSystem cs, cs1;

    public static double targetTicks = 0;
    public static double ticks;

    public static NormalizedColorSensor colorSensor;
    private Servo hood;

    int[] ballSlots = new int[]{0,0,0};
    int ballCount = 0;
    boolean colorPreviouslyDetected = false;
    boolean sorting = false;
    int[] sortTarget = new int[]{0,0,0};


    @Override
    public void runOpMode() {
        currentTimer=new Timer();

        cs1 = ControlSystem.builder()
                .posPid(p1,i1,d1)
                .build();
        cs = ControlSystem.builder()
                .velPid(p,i,d)
                .basicFF(v,a,s)
                .build();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        rconstants.initHardware(hardwareMap);

        lf = rconstants.lf;
        lb = rconstants.lb;
        rf = rconstants.rf;
        rb = rconstants.rb;

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretServo = hardwareMap.crservo.get("turretL");

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        configurePinpoint(pinpoint);

        flywheel = rconstants.flywheel;
        intake = rconstants.intake;
        spindexer = rconstants.spindexer;
        hood = rconstants.hood;
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transfer = rconstants.transfer;
        transfermover = rconstants.transfermover;
        transfermover.setPosition(rconstants.transfermoveridle);

        colorSensor = rconstants.colorSensor;
        distance = (DistanceSensor) colorSensor;

        cs1 = ControlSystem.builder().posPid(p1, i1, d1).build();
        cs = ControlSystem.builder().velPid(p, i, d).basicFF(v, a, s).build();

        Thread driveThread = new Thread(() -> {
            while (opModeIsActive()) {

                pinpoint.update();

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx = gamepad1.right_stick_x;

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                lf.setPower((y + x + rx) / denom);
                lb.setPower((y - x + rx) / denom);
                rf.setPower((y - x - rx) / denom);
                rb.setPower((y + x - rx) / denom);



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

                if (Math.abs(gamepad2.right_stick_x) < 0.05) {
                    turretPID = ControlSystem.builder()
                            .posPid(turretp, turreti, turretd)
                            .basicFF(turretv,turreta,turrets)
                            .build();

                    turretPID.setGoal(new KineticState(targetTicks));
                    turretServo.setPower(-turretPID.calculate(current));
                } else {
                    turretServo.setPower(gamepad2.right_stick_x);
                    targetTicks = turretEnc.getCurrentPosition()/10.0;
                }
            }
        });

        waitForStart();
        driveThread.start();
        int target = 0;

        while (opModeIsActive()) {
            KineticState current2 = new KineticState(spindexer.getCurrentPosition(), spindexer.getVelocity());
            cs1.setGoal(new KineticState(target));
            if(gamepad2.left_bumper){
                int moveamount = rconstants.movespindexer-(target%rconstants.movespindexer);
                target+=moveamount;
                sleep(300);
            }
            if (Math.abs(gamepad2.left_stick_y) < 0.1) {
                spindexer.setPower(-cs1.calculate(current2));
            } else{
                spindexer.setPower(0.6*gamepad2.left_stick_y);
                target = spindexer.getCurrentPosition();
            }

            if(gamepad2.right_trigger>0 && gamepad2.right_bumper){
                transfermover.setPosition(rconstants.transfermoverfull);
                transfer.setPower(gamepad2.right_trigger);
            }
            if(gamepad2.right_trigger>0 && !gamepad2.right_bumper){
                transfermover.setPosition(rconstants.transfermoverscore);
                transfer.setPower(gamepad2.right_trigger);
            }
            if(gamepad2.right_trigger==0){
                transfermover.setPosition(rconstants.transfermoveridle);
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
            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            KineticState current1 = new KineticState(flywheel.getCurrentPosition(),flywheel.getVelocity());
            flywheel.setPower(cs.calculate(current1));
            if (gamepad2.y) {
                targetTicksPerSecond = rconstants.shootfar;
                hood.setPosition(rconstants.hoodtop);
            }
            if (gamepad2.b){
                targetTicksPerSecond = rconstants.shootclose;
                hood.setPosition(rconstants.hoodtop);
            }
            if (gamepad2.a) {
                targetTicksPerSecond = rconstants.shooteridle;
                hood.setPosition(rconstants.hoodbottom);
            }
            if(gamepad2.right_stick_button){
                pinpoint.setPosX(144-21.5,DistanceUnit.INCH);
                pinpoint.setPosY(144-12.5,DistanceUnit.INCH);
                pinpoint.setHeading(35,AngleUnit.DEGREES);
                //*TARGET_Y=125.15254237288136;
                //*TARGET_X=121.89830508474577;
            }
            if (gamepad2.back) {
                autoalign = !autoalign;
                sleep(300);
            }

            telemetry.addData("AutoAlign", autoalign);
            telemetry.addData("Turret Pos", turretEnc.getCurrentPosition()/10.0);
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Robot Heading:" , pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("LL tx",
                    limelight.getLatestResult() != null
                            ? limelight.getLatestResult().getTx()
                            : 0
            );
            telemetry.update();
        }
    }

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
}
