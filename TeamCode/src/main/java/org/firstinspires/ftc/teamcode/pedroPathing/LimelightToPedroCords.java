package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class LimelightToPedroCords {
    /**
     * Converts given limelight cords to pedro pathing cords
     * @param limelightCords Limelight cords (eg. megatag 1 or 2)
     * @return Pedropathing Pose
     */
    public static Pose limelightToPedro(Pose2D limelightCords) {
        Pose ftcStandard = PoseConverter.pose2DToPose(limelightCords, InvertedFTCCoordinates.INSTANCE);
        Pose pedroPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        return pedroPose;
    }
}