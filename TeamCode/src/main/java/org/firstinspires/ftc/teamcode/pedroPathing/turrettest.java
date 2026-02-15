package org.firstinspires.ftc.teamcode.pedroPathing;

// Dashboard + Telemetry
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

// Sensors
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

// FTC
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// Navigation
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Control
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import java.util.List;

@Configurable
@Config
@TeleOp
public class turrettest extends LinearOpMode {

    // =====================
    // Hardware
    // =====================
    DcMotorEx turretEnc;
    CRServo turretServo;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

    // =====================
    // Control
    // =====================
    ControlSystem turretPID;

    // =====================
    // Constants
    // =====================
    public static double START_X = 8;
    public static double START_Y = 8.75;
    public static double START_HEADING_DEG = 0;

    public static double p = 0.0015;
    public static double i = 0;
    public static double d = 0.000000005;
    public static double v = 0.0000372;
    public static double a = 0.007;
    public static double s= 0.005;
    public static double ticksPerDegree = 126.42/10.0;

    // =====================
    // State
    // =====================
    public static double target = 0;

    @Override
    public void runOpMode() {

        // =====================
        // Telemetry
        // =====================
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );

        // =====================
        // Hardware Init
        // =====================
        turretEnc = hardwareMap.get(DcMotorEx.class, "turret_enc");
        turretServo = hardwareMap.crservo.get("turretL");

        turretEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pp");
        configurePinpoint(pinpoint);

        // =====================
        // PID Init (ONCE)
        // =====================


        waitForStart();

        // =====================
        // Post-start Pinpoint Setup
        // =====================
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setPosX(START_X, DistanceUnit.INCH);
        pinpoint.setPosY(START_Y, DistanceUnit.INCH);
        pinpoint.setHeading(START_HEADING_DEG, AngleUnit.DEGREES);

        int detected = 0;
        int actual = 0;

        // =====================
        // Main Loop
        // =====================
        while (opModeIsActive()) {

            // -------- Limelight --------
            LLResult llResult = limelight.getLatestResult();
            if (llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();
                detected = fiducials.get(0).getFiducialId();

                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (detected == 23) actual = 22;
                    else if (detected == 22) actual = 21;
                    else if (detected == 21) actual = 23;

                    telemetry.addData("Tag ID", fr.getFiducialId());
                }
            }

            // -------- Update Pinpoint --------
            pinpoint.update();
            turretPID = ControlSystem.builder()
                    .posPid(p, i, d)
                    .basicFF(v,a,s)
                    .build();

            // -------- Turret Target Math --------
            /*target = ticksPerDegree * (Math.toDegrees(
                    Math.atan2(
                            144-pinpoint.getPosY(DistanceUnit.INCH),
                            144-pinpoint.getPosX(DistanceUnit.INCH)
                    ))- (pinpoint.getHeading(AngleUnit.DEGREES)));
*/
            // -------- Manual Adjust --------
            if (gamepad2.dpad_right) {
                target += 5 * ticksPerDegree;
                sleep(300);
            }
            if (gamepad2.dpad_left) {
                target -= 5 * ticksPerDegree;
                sleep(300);
            }

            // -------- PID Control --------
            KineticState current =
                    new KineticState(
                            turretEnc.getCurrentPosition()/10.0,
                            turretEnc.getVelocity()
                    );

            turretPID.setGoal(new KineticState(target));

            if (Math.abs(gamepad2.left_stick_x) < 0.05) {

                turretServo.setPower(-turretPID.calculate(current));

            } else {
                turretServo.setPower(-gamepad2.left_stick_x);
                target = turretEnc.getCurrentPosition();
            }

            // -------- Telemetry --------
            telemetry.addData("Turret Power", -turretPID.calculate(current));
            telemetry.addData("Pinpoint X", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Turret Pos", turretEnc.getCurrentPosition()/10.0);
            telemetry.addData("Turret Target", target);
            telemetry.addData("Detected Tag", detected);
            telemetry.addData("Mapped Tag", actual);
            telemetry.addData("Turret Angle", Math.toDegrees(
                            Math.atan2(
                                    144-pinpoint.getPosY(DistanceUnit.INCH),
                                    144-pinpoint.getPosX(DistanceUnit.INCH)
                            )) - (pinpoint.getHeading(AngleUnit.DEGREES)));
            telemetry.update();
        }
    }

    // =====================
    // Pinpoint Config
    // =====================
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
