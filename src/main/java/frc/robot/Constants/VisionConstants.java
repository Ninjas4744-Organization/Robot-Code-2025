package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants.SimulationConstants;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.StateMachine.RobotState;

import java.util.Map;

public class VisionConstants {
    public static final com.ninjas4744.NinjasLib.DataClasses.VisionConstants kVisionConstants = new com.ninjas4744.NinjasLib.DataClasses.VisionConstants();
    static{
        kVisionConstants.cameras = Map.of(
                "Front", new Transform3d(0.3175, 0.0775, 0, new Rotation3d(0, Units.degreesToRadians(30), 0))
        );

        kVisionConstants.maxAmbiguity = 0.2;
        kVisionConstants.maxDistance = 6;
        kVisionConstants.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;

        kVisionConstants.simulationConstants = new SimulationConstants();
        kVisionConstants.simulationConstants.resolutionWidth = 1600;
        kVisionConstants.simulationConstants.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
        kVisionConstants.simulationConstants.resolutionHeight = 900;
        kVisionConstants.simulationConstants.FOV = 74;
        kVisionConstants.simulationConstants.averageError = 0.3;
        kVisionConstants.simulationConstants.errorStdDev = 0.5;
        kVisionConstants.simulationConstants.FPS = 30;
        kVisionConstants.simulationConstants.averageLatency = 35;
        kVisionConstants.simulationConstants.latencyStdDev = 5;
    }

    public static double calculateFOM(VisionOutput estimation) {
        double a = 0.55, b = 0.15, c = 2.2, d = 0.12;

        double distFOM = b * Math.exp(a * (estimation.closestTagDist - c)) / estimation.amountOfTargets + d;
        double speedFOM = 0.1 * RobotState.getInstance().getRobotVelocity().getNorm();

        return distFOM + speedFOM;
    };
}
