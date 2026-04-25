package org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight: Diagnostic Only", group = "Test")
public class ll_only extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // 2. Configure Limelight
        limelight.pipelineSwitch(5); // Switch to your desired pipeline
        limelight.start();

        telemetry.addData("Status", "Limelight Initialized. Waiting for Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 3. Get the latest result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Tracking data (tx, ty, ta)
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                Pose3D botpose = result.getBotpose();

                telemetry.addData("Target Found", "YES");
                telemetry.addData("TX (Deg)", "%.2f", tx);
                telemetry.addData("TY (Deg)", "%.2f", ty);
                telemetry.addData("Target Area (%)", "%.2f", ta);

                if (botpose != null) {
                    telemetry.addData("Botpose X (m)", "%.3f", botpose.getPosition().x);
                    telemetry.addData("Botpose Y (m)", "%.3f", botpose.getPosition().y);
                    telemetry.addData("Botpose Yaw (deg)", "%.2f", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                }
            } else {
                telemetry.addData("Target Found", "NO");
                telemetry.addLine("Check: Is a target in frame? Is Pipeline 5 configured?");
            }

            // Diagnostic Info
            telemetry.addData("Current Pipeline", result != null ? result.getPipelineIndex() : "N/A");
            telemetry.addData("LL Latency (ms)", result != null ? result.getCaptureLatency() + result.getTargetingLatency() : 0);
            telemetry.update();
        }

        // 4. Shutdown
        limelight.stop();
    }
}