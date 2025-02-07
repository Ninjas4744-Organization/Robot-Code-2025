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
//                "Front", new Transform3d(0.07, 0.105, -0.055, new Rotation3d(0, Units.degreesToRadians(30), 0)),
//                "Right", new Transform3d(-0.13, 0.145, -0.06, new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(-90 + 3)))
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
        double C = 0.116;

        double distFOM = (2 * Math.pow(2, estimation.closestTagDist)) / estimation.amountOfTargets * C;
        double speedFOM = 0/*0.2 * RobotState.getInstance().getRobotVelocity().getNorm()*/;

        return distFOM + speedFOM;
    };
}
