package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandBuilder;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class FieldConstants {
    public static final double kIntakeThreshold = 1.5;
    public static final double kXOuttakeDistThreshold = 0.03;
    public static final double kYOuttakeDistThreshold = 0.025;
    public static final double kOuttakeAngleThreshold = 2;
    public static final double kStartPIDThreshold = 2;

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

    public static AprilTag getTagFromPose(Pose2d pose) {
        List<AprilTag> tags = getFieldLayout().getTags();
        for (AprilTag tag : tags)
            if (tag.pose.toPose2d().equals(pose))
                return tag;

        return null;
    }

    public static Pose2d getOffsetTagPose(Pose2d tagPose, double offset) {
        return tagPose.transformBy(new Transform2d(offset, 0, new Rotation2d()));
    }

    public static Pose2d getClosestReefTag(){
        List<AprilTag> tags = getFieldLayoutWithAllowed(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22)).getTags();
        List<Pose2d> poses = new ArrayList<>();
        for(AprilTag tag : tags)
            poses.add(tag.pose.toPose2d());

        return RobotState.getInstance().getRobotPose().nearest(poses);
    }

    public static Pose2d getClosestReefTarget(boolean isRight, double extraChange){
        boolean sadna = false;
        if(sadna){
            return switch (RobotState.getInstance().getReefLevel()){
                case 4 -> getClosestReefTag().transformBy(new Transform2d(0.42/*0.45*/, isRight ? 0.16 + 0.03 + extraChange : -0.171 + extraChange, new Rotation2d()));
                case 3, 2 -> getClosestReefTag().transformBy(new Transform2d(0.42/*0.45*/, isRight ? 0.16 + extraChange : -0.16 + extraChange, new Rotation2d()));
                case 1 -> getClosestReefTag().transformBy(new Transform2d(0.42/*0.45*/, isRight ? 0.35 + extraChange : -0.35 + extraChange, new Rotation2d()));
                default -> throw new IllegalStateException("Unexpected value: " + RobotState.getInstance().getReefLevel());
            };
        }

        return switch (RobotState.getInstance().getReefLevel()){
            case 4 -> getClosestReefTag().transformBy(new Transform2d(0.42/*0.45*/, isRight ? 0.16 + extraChange : -0.171 - 0.03 + extraChange, new Rotation2d()));
            case 3, 2 -> getClosestReefTag().transformBy(new Transform2d(0.42/*0.45*/, isRight ? 0.16 + extraChange : -0.16 - 0.015 - 0.015 + extraChange, new Rotation2d()));
            case 1 -> getClosestReefTag().transformBy(new Transform2d(0.42/*0.45*/, isRight ? 0.35 + extraChange : -0.35 + extraChange, new Rotation2d()));
            default -> throw new IllegalStateException("Unexpected value: " + RobotState.getInstance().getReefLevel());
        };
    }

    public static boolean nearCoralStation(){
        return RobotState.getInstance().getDistance(FieldConstants.getTagPose(1).toPose2d()) < FieldConstants.kIntakeThreshold
        || RobotState.getInstance().getDistance(FieldConstants.getTagPose(2).toPose2d()) < FieldConstants.kIntakeThreshold
        || RobotState.getInstance().getDistance(FieldConstants.getTagPose(12).toPose2d()) < FieldConstants.kIntakeThreshold
        || RobotState.getInstance().getDistance(FieldConstants.getTagPose(13).toPose2d()) < FieldConstants.kIntakeThreshold;
    }

    public static int getAlgaeLevel() {
        return switch (getTagFromPose(getClosestReefTag()).ID) {
            case 6, 8, 10, 17, 19, 21 -> 1;
            case 7, 9, 11, 18, 20, 22 -> 2;
            default -> 1;
        };
    }
}
