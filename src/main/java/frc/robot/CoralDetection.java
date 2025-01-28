package frc.robot;
import frc.robot.Constants.CoralDetectionConstants;

public class CoralDetection {
    public enum DetectedCoral {
        NONE,
        BOTH,
        LEFT,
        RIGHT
    }

    public static DetectedCoral getCoralDetection() {
        double tx = LimelightHelpers.getTX("");
        double ta = LimelightHelpers.getTA("");

        if(ta < CoralDetectionConstants.kBottomAreaThreshold)
            return DetectedCoral.NONE;

        else if(ta > CoralDetectionConstants.kTopAreaThreshold)
            return DetectedCoral.BOTH;

        else {
            if(tx < -CoralDetectionConstants.kSideThreshold)
                return DetectedCoral.LEFT;
                
            else if(tx > CoralDetectionConstants.kSideThreshold)
                return DetectedCoral.RIGHT;
        }
        return DetectedCoral.NONE;
    }
}
