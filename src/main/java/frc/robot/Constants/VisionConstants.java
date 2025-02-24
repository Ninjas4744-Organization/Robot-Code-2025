package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants.SimulationConstants;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.StateMachine.RobotState;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

public class VisionConstants {
    public static final com.ninjas4744.NinjasLib.DataClasses.VisionConstants kVisionConstants = new com.ninjas4744.NinjasLib.DataClasses.VisionConstants();
    static{
        kVisionConstants.cameras = Map.of(
                "FrontRight", new Transform3d(0.2129+0.015+0.0805, -0.2015, -0.065, new Rotation3d(0, Units.degreesToRadians(-20), 0)),
                "FrontLeft", new Transform3d(0.27765+0.09+0.0805, 0.22965, -0.04573, new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(-15 + 4)))
        );

        kVisionConstants.maxAmbiguity = 0.2;
        kVisionConstants.maxDistance = 7;
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

    static InterpolatingDoubleTreeMap distFOMMap = new InterpolatingDoubleTreeMap();
    static{
        distFOMMap.put(0.0, 0.0);
        distFOMMap.put(0.5, 0.1);
        distFOMMap.put(1.0, 0.2);
        distFOMMap.put(3.0, 1.0);
        distFOMMap.put(6.0, 3.0);
        distFOMMap.put(7.0, 7.0);
        distFOMMap.put(8.0, 12.0);
    }
    public static double calculateFOM(VisionOutput estimation) {
//        double a = 1.33, b = 0.153, c = 4.839, d = 0.29;
//        double distFOM = (b * Math.exp(a * (estimation.closestTagDist - c)) + d) / estimation.amountOfTargets;

        double distFOM = distFOMMap.get(estimation.closestTagDist) / estimation.amountOfTargets;
        double speedFOM = 0;//0.1 * RobotState.getInstance().getRobotVelocity().getNorm();

        return distFOM + speedFOM;
    }
}
