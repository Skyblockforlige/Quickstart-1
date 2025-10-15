package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@TeleOp
public class teleop extends LinearOpMode {
    public static double p=0.019,i=0,d=3;
    public static double v=0.4,a=0.1,s=0.3;
    private ControlSystem cs;
    private DcMotorEx flywheel;
    private ServoImplEx turret;
    private ServoImplEx turret2;
    private DcMotorEx spindexer;
    private double targetx;
    private IMU imu;
    private int turretOscillationDirection;
    private Limelight3A limelight;
    private DcMotorEx intake;
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;
    private ColorSensor colorSensor;

    public static double targetTicksPerSecond=2200;
    @Override
    public void runOpMode(){
        colorSensor=hardwareMap.get(ColorSensor.class,"cs");
        // Create the PURPLE color locator
        ColorBlobLocatorProcessor purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 0, 255)) // magenta outline for purple blobs
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Create the GREEN color locator
        ColorBlobLocatorProcessor greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(0, 255, 0)) // green outline for green blobs
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // Add both processors to the same VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel = hardwareMap.get(DcMotorEx.class,"m");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake=hardwareMap.get(DcMotorEx.class,"intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer = hardwareMap.get(DcMotorEx.class,"spindexer");
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret = hardwareMap.get(ServoImplEx.class, "turret");
        turret2 = hardwareMap.get(ServoImplEx.class,"turret2");
        cs =  ControlSystem.builder()
                .velPid(p, i, d)
                .basicFF(v,a,s)
                .build();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret2.setPosition(0.5);
        turret.setPosition(0.5);

        waitForStart();
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);
            //intake code
            intake.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
            }

            // Turret code - detects AprilTag and goes to its location
            if (llResult != null) {
                targetx = llResult.getTx();
                telemetry.addData("targetx", llResult.getTx());
                if ((targetx <= -5.5) && (turret.getPosition() > 0.01)) {
                    turret.setPosition(turret.getPosition() - 0.01);
                    turret2.setPosition(turret.getPosition() - 0.01);
                    turretOscillationDirection=0;
                } else if ((targetx >= 5.5) && (turret.getPosition() < 0.99)) {
                    turret.setPosition(turret.getPosition() + 0.01);
                    turret2.setPosition(turret.getPosition() + 0.01);

                    turretOscillationDirection=1;
                }
            }

            if ((llResult.getTx() == 0.0) && (llResult.getTy() == 0.0)) {
                if ((turretOscillationDirection == 1) && (turret.getPosition() < 0.996)) {
                    turret.setPosition(turret.getPosition() + 0.01);
                    turret2.setPosition(turret2.getPosition() + 0.01);

                    if (turret.getPosition() >= 0.996) {
                        turret.setPosition(0.994);
                        turret2.setPosition(0.994);
                        turretOscillationDirection = 0; // now go left
                    }
                } else if ((turretOscillationDirection == 0) && (turret.getPosition() > 0.0040)) {
                    turret.setPosition(turret.getPosition() - 0.01);
                    turret2.setPosition(turret2.getPosition() - 0.01);

                    if (turret.getPosition() <= 0.0040) {
                        turret.setPosition(0.0045);
                        turret2.setPosition(0.0045);
                        turretOscillationDirection = 1; // now go right
                    }
                }
            }

            telemetry.addData("Xseen", llResult.getTx());
            telemetry.addData("turretPosition", turret.getPosition());
            telemetry.addData("turretOscillationDirection", turretOscillationDirection);

            cs.setGoal(new KineticState(0,targetTicksPerSecond));
            if(gamepad2.dpad_up){
                targetTicksPerSecond=2200;
            }
            if(gamepad2.dpad_down){
                targetTicksPerSecond=0;
            }
            KineticState current = new KineticState(flywheel.getCurrentPosition(), Math.abs(flywheel.getVelocity()));
            double output = cs.calculate(current);
            flywheel.setPower(output);

            telemetry.addData("motor current speed",Math.abs(flywheel.getVelocity()));
            telemetry.addData("target",targetTicksPerSecond);
            telemetry.addData("output",output);
            telemetry.update();

        }
    }
}
