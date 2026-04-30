package org.firstinspires.ftc.teamcode.pedroPathing.miscelenous_important_codes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OffsetsTuner extends OpMode {
    private Follower follower;
    
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72));
        follower.update();
    }

    /** This initializes the PoseUpdater as well as the Panels telemetry. */
    @Override
    public void init_loop() {
        telemetry.addLine("Prerequisite: Make sure both your offsets are set to 0 in your localizer constants.");
        telemetry.addLine("Turn your robot " + Math.PI + " radians. Your offsets in inches will be shown on the telemetry.");
        telemetry.update();
    }

    /**
     * This updates the robot's pose estimate, and updates the Panels telemetry with the
     * calculated offsets and draws the robot.
     */
    @Override
    public void loop() {
        follower.update();

        telemetry.addLine("Total Angle: " + follower.getTotalHeading());

        telemetry.addLine("The following values are the offsets in inches that should be applied to your localizer.");
        telemetry.addLine("strafeX: " + ((72.0-follower.getPose().getX()) / 2.0));
        telemetry.addLine("forwardY: " + ((72.0-follower.getPose().getY()) / 2.0));
        telemetry.update();
    }
}
