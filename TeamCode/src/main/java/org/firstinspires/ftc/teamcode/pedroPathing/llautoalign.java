package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;





@TeleOp
public class llautoalign extends OpMode {

    private Limelight3A limelight;
    private Servo s;
    private double targetx;
    private IMU imu;
    private int turretOscillationDirection;
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    double regularDivBy = 1;

    @Override
    public void init() {
        s = hardwareMap.servo.get("s");
        turretOscillationDirection = 0; // 0=left, 1=righ
        s.setPosition(0.5);
         lf = hardwareMap.dcMotor.get("m");
         lb = hardwareMap.dcMotor.get("lb");
         rf = hardwareMap.dcMotor.get("rf");
         rb = hardwareMap.dcMotor.get("rb");

        // limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
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

        // Turret code - detects AprilTag and goes to its location
        if (llResult != null) {
            targetx = llResult.getTx();
            telemetry.addData("targetx", llResult.getTx());
            if ((targetx <= -5.5) && (s.getPosition() > 0.01)) {
                s.setPosition(s.getPosition() - 0.01);
                turretOscillationDirection=0;
            } else if ((targetx >= 5.5) && (s.getPosition() < 0.99)) {
                s.setPosition(s.getPosition() + 0.01);
                turretOscillationDirection=1;
            }
        }

        if ((llResult.getTx() == 0.0) && (llResult.getTy() == 0.0)) {
            if ((turretOscillationDirection == 1) && (s.getPosition() < 0.99)) {
                s.setPosition(s.getPosition() + 0.01);
                if (s.getPosition() >= 0.996) {
                    s.setPosition(0.994);
                    turretOscillationDirection = 0; // now go left
                }
            } else if ((turretOscillationDirection == 0) && (s.getPosition() > 0.01)) {
                s.setPosition(s.getPosition() - 0.01);
                if (s.getPosition() <= 0.0040) {
                    s.setPosition(0.0045);
                    turretOscillationDirection = 1; // now go right
                }
            }
        }

        telemetry.addData("Xseen", llResult.getTx());
        telemetry.addData("turretPosition", s.getPosition());
        telemetry.addData("turretOscillationDirection", turretOscillationDirection);

        telemetry.update();
    }
}