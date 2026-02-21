package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.LinearHeadingPath;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.RTPAxon;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


@Configurable
@Config
@TeleOp
@Disabled
public class llautoalignred extends LinearOpMode {
    private double deg=0;
    private double tardeg=0;
    private Limelight3A limelight;
    double gear_rotio=200.0/38.0;
    private CRServo turretL;
    private CRServo turretR;
    private double targetx;
    private IMU imu;
    private int turretOscillationDirection;
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;
    private Timer goontimer;
    public static double multiplier=0.3333;

    double regularDivBy = 1;

    @Override
    public void runOpMode() {
        goontimer=new Timer();
        turretOscillationDirection = 0; // 0=left, 1=righ
        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        //ll.setDirection(CRServo.Direction.REVERSE);
        //5 on control hub
        turretL = hardwareMap.get(CRServo.class, "turretL");
        turretR = hardwareMap.get(CRServo.class, "turretR");
        waitForStart();

        limelight.start();
        goontimer.resetTimer();
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

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw());
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                telemetry.addData("Tx", llResult.getTx());
                telemetry.addData("Ty", llResult.getTy());
                telemetry.addData("Ta", llResult.getTa());
            }
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            // Turret code - detects AprilTag and goes to its location


            if (llResult != null) {
                targetx = llResult.getTx();
                telemetry.addData("targetx", llResult.getTx());

                if (targetx >= 5.5) {
                    // not necesary but makes it move exactly one degree
                    turretL.setPower(0.5);
                    turretR.setPower(0.5);
                    turretOscillationDirection = 0;
                    //switch to negative and make other postive if goes wrong direction
                } else if (targetx <= -5.5) {
                    turretL.setPower(-0.5);
                    turretR.setPower(-0.5);
                    turretOscillationDirection = 1;
                } else if(targetx>=-5.5&& targetx<=5.5){
                    turretR.setPower(0);
                    turretL.setPower(0);
                }
            } else{
                if(turretOscillationDirection == 0){
                    turretL.setPower(-0.1);
                    turretR.setPower(-0.1);
                    sleep(500);
                    turretOscillationDirection=1;
                } else{
                    turretL.setPower(0.1);
                    turretR.setPower(0.1);
                    sleep(500);
                    turretOscillationDirection=0;
                }
            }





            telemetry.addData("Xseen", llResult.getTx());
            telemetry.addData("turretOscillationDirection", turretOscillationDirection);
            telemetry.update();
        }

    }



}