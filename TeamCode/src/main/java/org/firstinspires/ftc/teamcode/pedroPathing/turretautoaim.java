package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Turret Fused Auto Aim")
public class turretautoaim extends OpMode {

    private Follower follower;
    private Limelight3A limelight;
    private CRServo turretL, turretR;
    private DcMotorEx turretEnc;

    // Constants synced with tele_w_TURRET
    public static double ticksPerDeg = 126.42;
    public static double turret_max = 90.0;
    public static double turret_min = -90.0;
    public static int servoDir = -1; 
    public static double kP_limelight = 0.015; // From kP_track in tele_w_TURRET
    public static double kP_arctan = 0.02;    // From kP_hold in tele_w_TURRET
    public static double maxPower = 0.3;

    // Target Field Position (Pinpointed Target)
    public static double targetX = 72; 
    public static double targetY = 72; 
    private boolean hasTargetLocked = false;

    /**
     * Distance calculation from Limelight Area (TA)
     */
    public double distancefromll(double ta) {
        if (ta <= 0) return 0;
        return (71.7321 * (Math.pow(ta, -0.4550)));
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rconstants.initHardware(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        
        limelight = rconstants.limelight;
        turretL = rconstants.turretL;
        turretR = rconstants.turretR;
        
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.setPollRateHz(100);
        limelight.start();

        telemetry.addLine("Initialized. Fused Limelight + ArcTan Aiming.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose robotPose = follower.getPose();
        double robotHeadingDeg = Math.toDegrees(robotPose.getHeading());
        double currentTurretDeg = turretEnc.getCurrentPosition() / ticksPerDeg;

        LLResult res = limelight.getLatestResult();
        boolean hasLimelightTarget = (res != null) && res.isValid();
        
        double cmdPower = 0;

        if (hasLimelightTarget) {
            // 1) DIRECT LIMELIGHT TRACKING
            double ty = res.getTy(); // Use TY as horizontal per tele_w_TURRET
            
            // Calculate power to center the target
            // In tele_w_TURRET: cmdPower = -kP_track * tyFilt
            cmdPower = -kP_limelight * ty;

            // 2) UPDATE PINPOINTED TARGET POSITION
            double distance = distancefromll(res.getTa());
            if (distance > 0) {
                // Angle to target in field coords = robot heading + turret relative + limelight offset
                // Note: If ty > 0 means target is to the right, then we ADD ty.
                double angleToTargetDeg = angleWrapDeg(robotHeadingDeg + currentTurretDeg + ty);
                double angleToTargetRad = Math.toRadians(angleToTargetDeg);

                targetX = robotPose.getX() + distance * Math.cos(angleToTargetRad);
                targetY = robotPose.getY() + distance * Math.sin(angleToTargetRad);
                hasTargetLocked = true;
            }
            
            telemetry.addData("Status", "TRACKING (Limelight)");
        } else if (hasTargetLocked) {
            // 3) ARCTAN FALLBACK
            double dx = targetX - robotPose.getX();
            double dy = targetY - robotPose.getY();
            
            double thetaFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
            double relNeededDeg = angleWrapDeg(thetaFieldDeg - robotHeadingDeg);
            
            double clampedTargetDeg = Math.max(turret_min, Math.min(turret_max, relNeededDeg));
            double error = angleWrapDeg(clampedTargetDeg - currentTurretDeg);
            
            cmdPower = error * kP_arctan;
            
            telemetry.addData("Status", "TRACKING (ArcTan)");
            telemetry.addData("Target Pos", "X: %.1f, Y: %.1f", targetX, targetY);
        } else {
            telemetry.addData("Status", "IDLE (Searching)");
        }

        // Apply global power clamping
        cmdPower = Math.max(-maxPower, Math.min(maxPower, cmdPower));

        // Soft limits
        if (currentTurretDeg >= turret_max && cmdPower > 0) cmdPower = 0;
        if (currentTurretDeg <= turret_min && cmdPower < 0) cmdPower = 0;

        setTurretPower(cmdPower);

        // Drive controls
        driveMecanums();

        telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.1f", 
                robotPose.getX(), robotPose.getY(), robotHeadingDeg);
        telemetry.addData("Turret Rel Angle", "%.2f", currentTurretDeg);
        telemetry.addData("Command Power", "%.3f", cmdPower);
        telemetry.update();
    }

    private void driveMecanums() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        rconstants.lf.setPower((y + x + rx) / denom);
        rconstants.lb.setPower((y - x + rx) / denom);
        rconstants.rf.setPower((y - x - rx) / denom);
        rconstants.rb.setPower((y + x - rx) / denom);
    }

    private void setTurretPower(double pwr) {
        // servoDir is -1, matches tele_w_TURRET logic
        turretL.setPower(pwr * servoDir);
        turretR.setPower(pwr * servoDir);
    }

    private double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
