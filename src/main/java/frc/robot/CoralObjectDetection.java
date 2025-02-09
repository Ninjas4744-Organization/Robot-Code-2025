package frc.robot;
import frc.robot.Constants.CoralDetectionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class CoralObjectDetection {
    private static PhotonCamera _camera;
    private static DetectedCoral _lastDetectedCoral = DetectedCoral.NONE;

    public enum DetectedCoral {
        NONE,
        BOTH,
        LEFT,
        RIGHT
    }

    static{
        // _camera = new PhotonCamera("Coral");
    }

    public static DetectedCoral getCoralDetection() {
        if(_camera == null)
            return DetectedCoral.NONE;

        List<PhotonPipelineResult> results = _camera.getAllUnreadResults();
        if(results.isEmpty())
            return _lastDetectedCoral;

        PhotonPipelineResult result = results.get(results.size() - 1);
        if(!result.hasTargets())
            return DetectedCoral.NONE;
        if(result.targets.size() > 1)
            return DetectedCoral.BOTH;

        PhotonTrackedTarget target = result.getBestTarget();
        double tx = target.getYaw();

        if(tx < -CoralDetectionConstants.kSideThreshold)
            return DetectedCoral.LEFT;

        else if(tx > CoralDetectionConstants.kSideThreshold)
            return DetectedCoral.RIGHT;

        return DetectedCoral.NONE;
    }
}
