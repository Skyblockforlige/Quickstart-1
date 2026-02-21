package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Configurable
@Config
@TeleOp
@Disabled
public class turretpinpointlltest extends LinearOpMode {

    private DcMotor lf, lb, rf, rb;
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
    public static double TARGET_X = 0;
    public static double TARGET_Y=144;

    public static double p2 = 0.00035;
    public static double i2 = 0.0000000005;
    public static double d2 = 0.0000000002;

    public static double ticksPerDegree = 126.42;

    public static double llAngleScale = 1.0;
    public static boolean autoalign = false;

    public static double p = 0.0039, i = 0, d = 0.0000005;
    public static double v = 0.000372, a = 0.7, s = 0.0000005;
    public static double p1 = 0.0009, i1 = 0, d1 = 0;

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

                ticks = fieldAngleDeg * ticksPerDegree;
                targetTicks = ticks;

                KineticState current =
                        new KineticState(
                                turretEnc.getCurrentPosition(),
                                turretEnc.getVelocity()
                        );

                if (Math.abs(gamepad2.right_stick_x) < 0.05) {
                    turretPID = ControlSystem.builder()
                            .posPid(p2, i2, d2)
                            .build();

                    turretPID.setGoal(new KineticState(targetTicks));
                    turretServo.setPower(-turretPID.calculate(current));
                } else {
                    turretServo.setPower(gamepad2.right_stick_x);
                    targetTicks = turretEnc.getCurrentPosition();
                }
            }
        });

        waitForStart();
        driveThread.start();

        while (opModeIsActive()) {

            if(gamepad2.options){
                pinpoint.setPosX(121.89830508474577,DistanceUnit.INCH);
                pinpoint.setPosY(125.15254237288136,DistanceUnit.INCH);
                pinpoint.setHeading(35,AngleUnit.DEGREES);
                //*TARGET_Y=125.15254237288136;
                //*TARGET_X=121.89830508474577;
            }
            if (gamepad2.back) {
                autoalign = !autoalign;
                sleep(300);
            }

            telemetry.addData("AutoAlign", autoalign);
            telemetry.addData("Turret Pos", turretEnc.getCurrentPosition());
            telemetry.addData("Target Ticks", targetTicks);
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
