package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.StateMachine.RobotState;

public class LimelightVision {
    public static void init(){
        LimelightHelpers.setCameraPose_RobotSpace("", 0, 0, 0, 0, 0, 0);
    }

    public static VisionOutput getVisionEstimation() {
        VisionOutput estimation = new VisionOutput();
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("");

        if (results.targets_Fiducials.length == 0) {
            estimation.hasTargets = false;
            return estimation;
        }

        estimation.hasTargets = true;
        estimation.amountOfTargets = results.targets_Fiducials.length;
        estimation.robotPose = RobotState.getAlliance() == DriverStation.Alliance.Blue ? LimelightHelpers.getBotPose2d_wpiBlue("") : LimelightHelpers.getBotPose2d_wpiRed("");
        estimation.timestamp = results.timestamp_RIOFPGA_capture;

        double farthestDistance = Double.MIN_VALUE;
        double closestDistance = Double.MAX_VALUE;

        AprilTag farthestTag = null;
        AprilTag closestTag = null;

        for (LimelightHelpers.LimelightTarget_Fiducial tag : results.targets_Fiducials) {
            double distance = tag.getTargetPose_CameraSpace2D().getTranslation().getNorm();

            if (distance > farthestDistance) {
                farthestDistance = distance;
                farthestTag = FieldConstants.getFieldLayout().getTags().get((int)tag.fiducialID);
            }
            if (distance < closestDistance) {
                closestDistance = distance;
                closestTag = FieldConstants.getFieldLayout().getTags().get((int)tag.fiducialID);
            }
        }

        estimation.maxAmbiguity = 0.2;
        estimation.farthestTagDist = farthestDistance;
        estimation.farthestTag = farthestTag;
        estimation.closestTagDist = closestDistance;
        estimation.closestTag = closestTag;

        return estimation;
    }
}
