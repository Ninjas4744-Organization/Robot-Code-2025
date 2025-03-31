package frc.robot.Constants;

import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
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

    public static final double kJoystickDeadband = 0.05;
    public static final boolean kInvertGyro = false;
    public static final boolean kDriverFieldRelative = true;

    public static final com.ninjas4744.NinjasLib.DataClasses.SwerveConstants kSwerveConstants = new com.ninjas4744.NinjasLib.DataClasses.SwerveConstants();
    static{
        kSwerveConstants.openLoop = false;
        kSwerveConstants.trackWidth = 0.685;
        kSwerveConstants.wheelBase = 0.685;
        kSwerveConstants.kinematics = new SwerveDriveKinematics(
          new Translation2d(SwerveConstants.kSwerveConstants.wheelBase / 2.0, SwerveConstants.kSwerveConstants.trackWidth / 2.0),
          new Translation2d(SwerveConstants.kSwerveConstants.wheelBase / 2.0, -SwerveConstants.kSwerveConstants.trackWidth / 2.0),
          new Translation2d(-SwerveConstants.kSwerveConstants.wheelBase / 2.0, SwerveConstants.kSwerveConstants.trackWidth / 2.0),
          new Translation2d(-SwerveConstants.kSwerveConstants.wheelBase / 2.0, -SwerveConstants.kSwerveConstants.trackWidth / 2.0)
        );

        kSwerveConstants.maxSpeed = 4.5;//4.5;
        kSwerveConstants.maxAngularVelocity = 9.2;
//        kSwerveConstants.maxAcceleration = Double.MAX_VALUE;
//        kSwerveConstants.maxSkidAcceleration = Double.MAX_VALUE;
        kSwerveConstants.speedLimit = 4.5;//4.5;
        kSwerveConstants.rotationSpeedLimit = 9.2;
        kSwerveConstants.accelerationLimit = Double.MAX_VALUE;//11.8;//10;
        kSwerveConstants.rotationAccelerationLimit = Double.MAX_VALUE;//63.79;

        kSwerveConstants.enableLogging = true;
        kSwerveConstants.moduleConstants = new SwerveModuleConstants[4];

        double wheelRadius = 0.049806;//0.048;

        for(int i = 0; i < 4; i++){
            kSwerveConstants.moduleConstants[i] = new SwerveModuleConstants<>(i,
                    new MainControllerConstants(),
                    new MainControllerConstants(),
                    kSwerveConstants.maxSpeed,
                    6 + i,
                    NinjasTalonFXController.class,
                    NinjasTalonFXController.class,
                    false,
                    false,
                    0);

            kSwerveConstants.moduleConstants[i].driveMotorConstants.main.id = 10 + i * 2;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.currentLimit = 72;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.encoderConversionFactor = wheelRadius * 2 * Math.PI / 5.360;//0.056267331109070916;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.subsystemName = "Swerve Module " + i + " Drive Motor";
            kSwerveConstants.moduleConstants[i].driveMotorConstants.enableLogging = false;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.controlConstants = ControlConstants.createTorqueCurrent(5 / 0.056267331109070916, 0.19);

            kSwerveConstants.moduleConstants[i].angleMotorConstants.main.id = 11 + i * 2;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.currentLimit = 60;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.encoderConversionFactor = 19.2;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.subsystemName = "Swerve Module " + i + " Angle Motor";
            kSwerveConstants.moduleConstants[i].angleMotorConstants.enableLogging = false;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.controlConstants = ControlConstants.createPID(4 / 19.2, 0, 0, 0);
        }

        kSwerveConstants.moduleConstants[0].CANCoderOffset = -0.295654;
        kSwerveConstants.moduleConstants[1].CANCoderOffset = 0.230713;
        kSwerveConstants.moduleConstants[2].CANCoderOffset = 0.238037;
        kSwerveConstants.moduleConstants[3].CANCoderOffset = 0.273438;
    }

    public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
    static {
        kSwerveControllerConstants.swerveConstants = kSwerveConstants;
        kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(6, 0, 0.2, 0);
//        kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(0.01, 0.05, 0, 5);
        kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(0.2, 0, 0, 0);
        kSwerveControllerConstants.axisLockPIDConstants = ControlConstants.createPID(6, 0, 0.2, 0);
        kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-180.0, 180.0);
        kSwerveControllerConstants.driveAssistThreshold = 1.5;
        kSwerveControllerConstants.pathConstraints = new PathConstraints(2, 2, 8, 10.69886608888889);

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
