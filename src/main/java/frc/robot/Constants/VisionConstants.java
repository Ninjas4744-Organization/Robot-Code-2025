package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants.SimulationConstants;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
                "FrontRight", Pair.of(new Transform3d(0.0815+0.1054, -0.0745, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(-7.5-1.5))), com.ninjas4744.NinjasLib.DataClasses.VisionConstants.CameraType.PhotonVision),
                "FrontLeft", Pair.of(new Transform3d(0.0815+0.1054, 0.0755, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(7.5-1.5))), com.ninjas4744.NinjasLib.DataClasses.VisionConstants.CameraType.PhotonVision)
//                "Back", Pair.of(new Transform3d(0.0815+0.1054, 0.0755, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(180))), com.ninjas4744.NinjasLib.DataClasses.VisionConstants.CameraType.Limelight)
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
        distFOMMap.put(0.0, 0.2);
        distFOMMap.put(0.5, 0.5);
        distFOMMap.put(1.0, 1.0);
        distFOMMap.put(3.0, 4.0);
        distFOMMap.put(5.0, 80.0);
    }
    public static double[] calculateFOM(VisionOutput estimation) {
//        double a = 1.33, b = 0.153, c = 4.839, d = 0.29;
//        double distFOM = (b * Math.exp(a * (estimation.closestTagDist - c)) + d) / estimation.amountOfTargets;

        double distFOM = distFOMMap.get(estimation.closestTagDist) / estimation.amountOfTargets;
        double speedFOM = new Translation2d(
                SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond,
                SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond).getNorm() / 3;
        double FOM = distFOM + speedFOM;
        FOM /= estimation.cameraName.equals("Back") ? 2 : 1;

        return new double[] { FOM, FOM, FOM };
    }
}
