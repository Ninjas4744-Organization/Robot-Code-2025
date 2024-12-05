package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SwerveModuleConstants;

public class SwerveConstants {
    public static final double kSpeedFactor = 0.5;
    public static final double kRotationSpeedFactor = 0.25;
    public static final double kJoystickDeadband = 0.2;
    public static final boolean kInvertGyro = false;
    public static final boolean kFieldRelative = true;

    public static final com.ninjas4744.NinjasLib.DataClasses.SwerveConstants kSwerveConstants = new com.ninjas4744.NinjasLib.DataClasses.SwerveConstants();
    static{
        kSwerveConstants.openLoop = true;
        kSwerveConstants.trackWidth = 0.62;
        kSwerveConstants.wheelBase = 0.62;

        kSwerveConstants.maxSpeed = 5;
        kSwerveConstants.maxAngularVelocity = 10.7;
        kSwerveConstants.simulationAcceleration = 12;
        kSwerveConstants.simulationAngleAcceleration = 18;

        kSwerveConstants.canCoderInvert = false;
        kSwerveConstants.moduleConstants = new SwerveModuleConstants[4];

        for(int i = 0; i < 4; i++){
            kSwerveConstants.moduleConstants[i] = new SwerveModuleConstants(i, new MainControllerConstants(), new MainControllerConstants(), kSwerveConstants.maxSpeed, 0);
            kSwerveConstants.moduleConstants[i].driveMotorConstants.main.inverted = true;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.currentLimit = 50;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.encoderConversionFactor = 0.0521545447;
            kSwerveConstants.moduleConstants[i].driveMotorConstants.subsystemName = "Swerve Module " + i + " Drive Motor";
            kSwerveConstants.moduleConstants[i].driveMotorConstants.createShuffleboard = false;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.currentLimit = 50;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.encoderConversionFactor = 28.125;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.subsystemName = "Swerve Module " + i + " Angle Motor";
            kSwerveConstants.moduleConstants[i].angleMotorConstants.createShuffleboard = false;
            kSwerveConstants.moduleConstants[i].angleMotorConstants.controlConstants = ControlConstants.createPID(0.01, 0, 0.005, 0);
            kSwerveConstants.moduleConstants[i].maxModuleSpeed = kSwerveConstants.maxSpeed;
        }
        kSwerveConstants.moduleConstants[0].driveMotorConstants.main.id = 10;
        kSwerveConstants.moduleConstants[0].angleMotorConstants.main.id = 11;
        kSwerveConstants.moduleConstants[0].canCoderID = 40;

        kSwerveConstants.moduleConstants[1].driveMotorConstants.main.id = 12;
        kSwerveConstants.moduleConstants[1].angleMotorConstants.main.id = 13;
        kSwerveConstants.moduleConstants[1].canCoderID = 41;

        kSwerveConstants.moduleConstants[2].driveMotorConstants.main.id = 14;
        kSwerveConstants.moduleConstants[2].angleMotorConstants.main.id = 15;
        kSwerveConstants.moduleConstants[2].canCoderID = 42;

        kSwerveConstants.moduleConstants[3].driveMotorConstants.main.id = 16;
        kSwerveConstants.moduleConstants[3].angleMotorConstants.main.id = 17;
        kSwerveConstants.moduleConstants[3].canCoderID = 43;
    }
}
