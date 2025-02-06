package frc.robot;
import frc.robot.Constants.CoralDetectionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralObjectDetection {
    static PhotonCamera camera;

    public enum DetectedCoral {
        NONE,
        BOTH,
        LEFT,
        RIGHT
    }

    static{
        camera = new PhotonCamera("photonvision");
    }

    public static DetectedCoral getCoralDetection() {
        PhotonPipelineResult result = camera.getAllUnreadResults().get(0);
        PhotonTrackedTarget target = result.getBestTarget();
        double ta = target.getArea();
        double tx = target.getYaw();

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
