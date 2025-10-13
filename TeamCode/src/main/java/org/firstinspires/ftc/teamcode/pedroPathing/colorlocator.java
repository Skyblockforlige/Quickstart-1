package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import java.util.List;
@TeleOp(name = "Artifact Locator green+purple")
public class colorlocator extends LinearOpMode {
    @Override
    public void runOpMode() {
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

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Detecting Green and Purple Artifacts...");

            // Get purple blobs
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, purpleBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, purpleBlobs);

            // Get green blobs
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, greenBlobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, greenBlobs);

            List<ColorBlobLocatorProcessor.Blob> allBlobs = new java.util.ArrayList<>();
            allBlobs.addAll(purpleBlobs);
            allBlobs.addAll(greenBlobs);

            // Sort by X coordinate (left → right)
            allBlobs.sort((a, b) -> Double.compare(a.getCircle().getX(), b.getCircle().getX()));

            // --- Display sorted results ---
            telemetry.addLine("Color   X      Y     Radius   Circularity");
            for (ColorBlobLocatorProcessor.Blob blob : allBlobs) {
                Circle c = blob.getCircle();
                boolean isPurple = purpleBlobs.contains(blob);

                telemetry.addLine(String.format("%s   %3d   %3d     %3d       %.2f",
                        (isPurple ? "Purple" : "Green"),
                        (int) c.getX(), (int) c.getY(), (int) c.getRadius(), blob.getCircularity()));
            }

            telemetry.update();
            sleep(100);

        }
    }
}
