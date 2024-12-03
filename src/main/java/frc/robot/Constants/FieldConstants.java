package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.StateMachine.RobotState;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class FieldConstants {
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try{
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }catch(IOException e){
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public static AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags) {
        AprilTagFieldLayout layout;

        layout = RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue
            ? kBlueFieldLayout
            : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) layout.getTags().removeIf(tag -> ignoredTags.contains(tag.ID));

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayout() {
        return getFieldLayout(List.of());
    }

    public static AprilTag getAmpTag() {
        if (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue)
            return getFieldLayout().getTags().get(6 - 1);
        else return getFieldLayout().getTags().get(5 - 1);
    }

    public static AprilTag getSourceTag() {
        if (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue)
            return getFieldLayout().getTags().get(2 - 1);
        else return getFieldLayout().getTags().get(9 - 1);
    }

    public static AprilTag getSpeakerTag() {
        if (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue)
            return getFieldLayout().getTags().get(7 - 1);
        else return getFieldLayout().getTags().get(4 - 1);
    }

    public static Pose3d getTagPose(int id) {
        return getFieldLayout().getTagPose(id).get();
    }

    /**
     * Get the april tag that the translation between it and the robot is closest to the given direction,
     * for example if robot is moving to amp, the amp tag will be returned
     * @param dir The direction to find the closest camera to, field relative
     * @param distLimit Limit distance so far away tags will be disqualified
     * @return The apriltag that is closest by direction
     */
    public static AprilTag getTagByDirection(Translation2d dir, double distLimit) {
        Rotation2d dirAngle = dir.getAngle();
        AprilTag closestTag = null;
        Rotation2d closestAngleDiff = Rotation2d.fromDegrees(Double.MAX_VALUE);

        for (AprilTag tag : getFieldLayout().getTags()) {
            if (tag.pose
                .toPose2d()
                .getTranslation()
                .getDistance(RobotState.getInstance().getRobotPose().getTranslation())
                > distLimit) continue;

            Rotation2d robotToTagAngle = tag.pose
                .toPose2d()
                .getTranslation()
                .minus(RobotState.getInstance().getRobotPose().getTranslation())
                .getAngle();

            if (Math.abs(dirAngle.minus(robotToTagAngle).getDegrees()) < Math.abs(closestAngleDiff.getDegrees())) {
                closestAngleDiff = dirAngle.minus(robotToTagAngle);
                closestTag = tag;
            }
        }

        return closestTag;
    }

    /**
     * Get the april tag that the translation between it and the robot is closest to the given direction,
     * for example if robot is moving to amp, the amp tag will be returned
     *
     * @param dir The direction to find the closest camera to, field relative
     * @return The apriltag that is closest by direction
     */
    public static AprilTag getTagByDirection(Translation2d dir) {
        return getTagByDirection(dir, Double.MAX_VALUE);
    }

    /**
     * Get offset april tag pose according to its looking direction
     * @param offset how much to offset the pose of the tag to its looking direction
     * @return the pose of the offset tag
     */
    public static Pose2d getOffsetTagPose(Pose2d tagPose, double offset) {
        return tagPose.transformBy(new Transform2d(offset, 0, new Rotation2d()));
    }
}
