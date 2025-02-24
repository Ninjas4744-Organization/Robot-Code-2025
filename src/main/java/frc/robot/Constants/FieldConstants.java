package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandBuilder;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

import java.io.IOException;
import java.util.List;

public class FieldConstants {
    public static final double kIntakeThreshold = 2.5;

    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;
    static {
        try{
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
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

        if (!ignoredTags.isEmpty()){
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> ignoredTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayoutWithAllowed(List<Integer> allowedTags) {
        AprilTagFieldLayout layout = getFieldLayout();
        if (!allowedTags.isEmpty()){
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> !allowedTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

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

    public static Pose2d getClosestReefTag(){
        List<AprilTag> tags = getFieldLayoutWithAllowed(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)).getTags();

        AprilTag closestTag = null;
        double closestTagDistance = Double.MAX_VALUE;
        for(AprilTag tag : tags){
            Pose2d tagPose = tag.pose.toPose2d();
            double distance = RobotState.getInstance().getDistanceTo(tagPose).getNorm();

            if(distance < closestTagDistance){
                closestTag = tag;
                closestTagDistance = distance;
            }
        }

        return getOffsetTagPose(closestTag.pose.toPose2d(), 0.423);
    }

    public static Pose2d getOffsetReefTagPose(Pose2d tagPose, boolean isRight, boolean isL4){
        return tagPose.transformBy(new Transform2d(0, !isL4 ? (isRight ? 0.18 + 0.07 - 0.03 - 0.01 : -0.19 + 0.07 - 0.03 + 0.01 - 0.01) : (isRight ? 0.12 + 0.03 + 0.01 + 0.01 + 0.03 : -0.25 + 0.03 + 0.03 + 0.01), new Rotation2d()));
    }

    public static boolean nearCoralStation(){
        return RobotState.getInstance().getDistanceTo(FieldConstants.getTagPose(1).toPose2d()).getNorm() < FieldConstants.kIntakeThreshold
        || RobotState.getInstance().getDistanceTo(FieldConstants.getTagPose(2).toPose2d()).getNorm() < FieldConstants.kIntakeThreshold
        || RobotState.getInstance().getDistanceTo(FieldConstants.getTagPose(12).toPose2d()).getNorm() < FieldConstants.kIntakeThreshold
        || RobotState.getInstance().getDistanceTo(FieldConstants.getTagPose(13).toPose2d()).getNorm() < FieldConstants.kIntakeThreshold;
    }
}
