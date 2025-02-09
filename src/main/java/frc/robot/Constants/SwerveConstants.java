package frc.robot.Constants;

import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SwerveControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SwerveModuleConstants;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class SwerveConstants {
    public static final double kDriverSpeedFactor = 1;
    public static final double kDriverRotationSpeedFactor = 1;
    public static final double kJoystickDeadband = 0.1;
    public static final boolean kInvertGyro = false;

    public static final com.ninjas4744.NinjasLib.DataClasses.SwerveConstants kSwerveConstants = new com.ninjas4744.NinjasLib.DataClasses.SwerveConstants();
    static{
        kSwerveConstants.openLoop = true;
        kSwerveConstants.trackWidth = 0.7;
        kSwerveConstants.wheelBase = 0.7;
        kSwerveConstants.kinematics = new SwerveDriveKinematics(
          new Translation2d(SwerveConstants.kSwerveConstants.wheelBase / 2.0, SwerveConstants.kSwerveConstants.trackWidth / 2.0),
          new Translation2d(SwerveConstants.kSwerveConstants.wheelBase / 2.0, -SwerveConstants.kSwerveConstants.trackWidth / 2.0),
          new Translation2d(-SwerveConstants.kSwerveConstants.wheelBase / 2.0, SwerveConstants.kSwerveConstants.trackWidth / 2.0),
          new Translation2d(-SwerveConstants.kSwerveConstants.wheelBase / 2.0, -SwerveConstants.kSwerveConstants.trackWidth / 2.0)
        );

        kSwerveConstants.maxSpeed = 5;
        kSwerveConstants.maxAngularVelocity = 10.7;
        kSwerveConstants.speedLimit = 5;
        kSwerveConstants.rotationSpeedLimit = 10.7;
        kSwerveConstants.accelerationLimit = 15;//15
        kSwerveConstants.rotationAccelerationLimit = 54;//54

        kSwerveConstants.enableLogging = true;
        kSwerveConstants.moduleConstants = new SwerveModuleConstants[4];

        for(int i = 0; i < 4; i++){
            kSwerveConstants.moduleConstants[i] = new SwerveModuleConstants<>(i, new MainControllerConstants(), new MainControllerConstants(), kSwerveConstants.maxSpeed, 40 + i, NinjasSparkMaxController.class,  NinjasSparkMaxController.class, true, false, 0);
            kSwerveConstants.moduleConstants[i].driveMotorConstants.main.id = 10 + i * 2;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.main.inverted = true;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.currentLimit = 68;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.encoderConversionFactor = 0.0521545447;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.subsystemName = "Swerve Module " + i + " Drive Motor";
            kSwerveConstants.moduleConstants[i].driveMotorConstants.enableLogging = false;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.controlConstants = ControlConstants.createTorqueCurrent(5, 0.1);

            kSwerveConstants.moduleConstants[i].angleMotorConstants.main.id = 11 + i * 2;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.currentLimit = 50;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.encoderConversionFactor = 28.125;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.subsystemName = "Swerve Module " + i + " Angle Motor";
            kSwerveConstants.moduleConstants[i].angleMotorConstants.enableLogging = false;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.controlConstants = ControlConstants.createPID(0.01, 0, 0.005, 0);
        }

        kSwerveConstants.moduleConstants[0].CANCoderOffset = 0.493652;
        kSwerveConstants.moduleConstants[1].CANCoderOffset = -0.359375;
        kSwerveConstants.moduleConstants[2].CANCoderOffset = -0.270752;
        kSwerveConstants.moduleConstants[3].CANCoderOffset = -0.134277;
    }

    public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
    static {
        kSwerveControllerConstants.swerveConstants = kSwerveConstants;
        kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(2, 0, 0, 0);
        kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(0.057, 0.09, 0.003, 10);
        kSwerveControllerConstants.axisLockPIDConstants = ControlConstants.createPID(0.14, 0, 0, 0);
        kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-180.0, 180.0);
        kSwerveControllerConstants.driveAssistThreshold = 1.5;
        kSwerveControllerConstants.driverFieldRelative = false;
        kSwerveControllerConstants.pathConstraints = new PathConstraints(5, 10, 8, 50);

        try {
            kSwerveControllerConstants.robotConfig = RobotConfig.fromGUISettings();
        } catch (IOException e) {
            throw new RuntimeException(e);
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
    }

    public static final PathFollowingController kAutonomyConfig =
      new PPHolonomicDriveController(
        new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
        new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
      );
}
