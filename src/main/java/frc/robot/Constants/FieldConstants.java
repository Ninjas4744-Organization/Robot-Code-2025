package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.StateMachine.RobotState;

import java.io.IOException;
import java.util.List;

public class FieldConstants {
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try{
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }catch(IOException e){
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public static AprilTagFieldLayout getFieldLayoutWithIgnored(List<Integer> ignoredTags) {
        AprilTagFieldLayout layout;

        layout = RobotState.getAlliance() == DriverStation.Alliance.Blue
            ? kBlueFieldLayout
            : kRedFieldLayout;

        if (!ignoredTags.isEmpty())
            layout.getTags().removeIf(tag -> ignoredTags.contains(tag.ID));

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayoutWithAllowed(List<Integer> allowedTags) {
        AprilTagFieldLayout layout = getFieldLayout();
        if (!allowedTags.isEmpty())
            layout.getTags().removeIf(tag -> !allowedTags.contains(tag.ID));

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayout() {
        return getFieldLayoutWithIgnored(List.of());
    }

    public static Pose3d getTagPose(int id) {
        return getFieldLayout().getTagPose(id).get();
    }

    public static Pose2d getOffsetTagPose(Pose2d tagPose, double offset) {
        return tagPose.transformBy(new Transform2d(offset, 0, new Rotation2d()));
    }

    public static AprilTag getClosestReefTag(){
        List<AprilTag> tags = getFieldLayoutWithAllowed(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)).getTags();

        AprilTag closestTag = null;
        double closestTagDistance = Double.MAX_VALUE;
        for(AprilTag tag : tags){
            Translation2d robotPose = RobotState.getInstance().getRobotPose().getTranslation();
            Translation2d tagPose = getTagPose(tag.ID).toPose2d().getTranslation();
            double distance = robotPose.getDistance(tagPose);

            if(distance < closestTagDistance){
                closestTag = tag;
                closestTagDistance = distance;
            }
        }

        return closestTag;
    }

    public static Pose2d getOffsetReefTagPose(AprilTag tag, boolean isRight){
        return tag.pose.toPose2d().transformBy(new Transform2d(0, isRight ? -0.5 : 0.5, new Rotation2d()));
    }
}
