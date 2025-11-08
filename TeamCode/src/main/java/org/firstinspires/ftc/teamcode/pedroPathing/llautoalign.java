package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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





@TeleOp
public class llautoalign extends OpMode {
    private double deg=0;
    private double tardeg=0;
    private Limelight3A limelight;
    double gear_rotio=200.0/38.0;
    private RTPAxon axon1;
    private RTPAxon axon2;
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

        turretOscillationDirection = 0; // 0=left, 1=righ
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
        CRServo servo1 = hardwareMap.get(CRServo.class, "axon1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "axon2");
        AnalogInput encoder1 = hardwareMap.get(AnalogInput.class, "axon1");
        AnalogInput encoder2 = hardwareMap.get(AnalogInput.class, "axon2");
        RTPAxon axon1 = new RTPAxon(servo1, encoder1);
        RTPAxon axon2 = new RTPAxon(servo2, encoder2);
        axon1.setMaxPower(0.5);
        axon2.setMaxPower(0.5);
        axon1.setPidCoeffs(0.02, 0.0005, 0.0025);
        axon2.setPidCoeffs(0.02, 0.0005, 0.0025);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
         deg = axon1.getTotalRotation() / (200.0/38.0);
         tardeg = axon1.getTargetRotation() / (200.0/38.0);
        axon1.update();
        axon2.update();
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

                if (targetx >= 5.5) {
                    axon1.changeTargetRotation(-(gear_rotio));
                    axon2.changeTargetRotation(-(gear_rotio));// not necesary but makes it move axeactly one degree
                    turretOscillationDirection = 0;
                    //switch to negative and make other postive if goes wrong direction
                }

                if (targetx <= -5.5) {
                    axon1.changeTargetRotation((gear_rotio));
                    axon2.changeTargetRotation((gear_rotio));
                    turretOscillationDirection = 1;
                }
            }

            //if the limelight cant see no april tag it sweeps based on the last known direction
            if (llResult.getTx() == 0.0 && llResult.getTy() == 0.0) {
                if (turretOscillationDirection == 1) {
                    axon1.changeTargetRotation(1);
                    axon2.changeTargetRotation(1);
                    //might have to switch the direction
                }

                if (turretOscillationDirection == 0) {
                    axon1.changeTargetRotation(-1);
                    axon2.changeTargetRotation(-1);
                }
            }



            telemetry.addData("Xseen", llResult.getTx());
        telemetry.addData("turretOscillationDirection", turretOscillationDirection);
        //copy pasted from github
        telemetry.addData("Servo Position", axon1.getCurrentAngle());
        telemetry.addData("Total Rotation", axon1.getTotalRotation());
        telemetry.addData("Target Rotation", axon1.getTargetRotation());
        telemetry.update();

        telemetry.update();
    }
}