package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import frc.robot.StateMachine.StateMachine;
import frc.robot.Subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CommandBuilder {
    public static class Teleop{
        public static Command swerveDrive(
            Supplier<Translation2d> translation,
            Supplier<Translation2d> rotation,
            BooleanSupplier isLookAt,
            BooleanSupplier isBayblade) {
            return Commands.runOnce(
                () -> {
                    double lx = -MathUtil.applyDeadband(translation.get().getX(), SwerveConstants.kJoystickDeadband);
                    double ly = -MathUtil.applyDeadband(translation.get().getY(), SwerveConstants.kJoystickDeadband);
                    double rx = -MathUtil.applyDeadband(rotation.get().getX(), SwerveConstants.kJoystickDeadband);
                    double ry = -MathUtil.applyDeadband(rotation.get().getY(), SwerveConstants.kJoystickDeadband);

                    double finalRotation = rx * SwerveConstants.kDriverRotationSpeedFactor;

                    if (isLookAt.getAsBoolean())
                        finalRotation = SwerveController.getInstance().lookAt(new Translation2d(ry, rx), 45);

                    if (isBayblade.getAsBoolean())
                        finalRotation = 1;

                    SwerveController.getInstance().Demand.driverInput = new ChassisSpeeds(
                            ly * SwerveConstants.kDriverSpeedFactor,
                            lx * SwerveConstants.kDriverSpeedFactor,
                            finalRotation);
                }, SwerveSubsystem.getInstance());
        }

        public static Command resetGyro(boolean forceZero) {
            return Commands.runOnce(() -> {
                if (forceZero) RobotState.getInstance().resetGyro(Rotation2d.fromDegrees(0));
                else {
                    if (VisionIO.getInstance().hasTargets())
                        RobotState.getInstance().resetGyro(RobotState.getInstance().getRobotPose().getRotation());
                    else RobotState.getInstance().resetGyro(Rotation2d.fromDegrees(0));
                }
            });
        }

        public static Command changeRobotState(RobotStates state) {
            return Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(state), StateMachine.getInstance());
        }

        public static Command runIfTestMode(Command command) {
            return Commands.either(
                command,
                Commands.none(),
                () -> RobotState.getInstance().getRobotState() == RobotStates.TEST
            );
        }

        public static Command runIfNotTestMode(Command command) {
            return Commands.either(
                command,
                Commands.none(),
                () -> RobotState.getInstance().getRobotState() != RobotStates.TEST
            );
        }
    }

    public static class Auto{
        public static void configureAutoBuilder() {
            AutoBuilder.configure(
                () -> new Pose2d(
                    RobotState.getInstance().getRobotPose().getX(),
                    RobotState.getInstance().getRobotPose().getY() * (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
                    RobotState.getInstance().getRobotPose().getRotation().rotateBy(Rotation2d.fromDegrees(RobotState.getInstance().getAlliance() == DriverStation.Alliance.Red ? 180 : 0))
                ), // Robot pose supplier

                (pose) -> {
                    RobotState.getInstance().setRobotPose(new Pose2d(
                        pose.getX(),
                        pose.getY() * (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
                        pose.getRotation().rotateBy(Rotation2d.fromDegrees(RobotState.getInstance().getAlliance() == DriverStation.Alliance.Red ? 180 : 0))
                    ));
                    RobotState.getInstance().resetGyro(pose.getRotation());
                }, // Method to reset odometry (will be called if your auto has a starting pose)

                () -> SwerveIO.getInstance().getChassisSpeeds(false), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                (drive) -> SwerveController.getInstance().Demand.velocity = drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

                SwerveConstants.kAutonomyConfig, //Autonomy config

                SwerveConstants.kSwerveControllerConstants.robotConfig, //Robot config

                () -> false/*RobotState.getInstance().getAlliance() == Alliance.Red*/, // Boolean supplier that mirrors path if red alliance

                SwerveSubsystem.getInstance() // Reference to swerve subsystem to set requirements
            );

            registerCommands();
        }

        /** Registers all auto commands to pathplanner */
        private static void registerCommands() {
            NamedCommands.registerCommand("L1", L1());
            NamedCommands.registerCommand("L2", L2());
            NamedCommands.registerCommand("L3", L3());
            NamedCommands.registerCommand("L4", L4());
            NamedCommands.registerCommand("Intake", intake());
        }

        public static Command L1() {
            return Commands.run(() -> StateMachine.getInstance().changeRobotState(RobotStates.L1));
        }

        public static Command L2() {
            return Commands.run(() -> StateMachine.getInstance().changeRobotState(RobotStates.L2));
        }

        public static Command L3() {
            return Commands.run(() -> StateMachine.getInstance().changeRobotState(RobotStates.L3));
        }

        public static Command L4() {
            return Commands.run(() -> StateMachine.getInstance().changeRobotState(RobotStates.L4));
        }

        public static Command intake() {
            return Commands.run(() -> StateMachine.getInstance().changeRobotState(RobotStates.INTAKE));
        }
        

        /**
         * @return final autonomy command from pathplanner
         */
        public static Command autoCommand(String auto) {
            SwerveController.getInstance().setState(SwerveDemand.SwerveState.VELOCITY);
            SwerveController.getInstance().Demand.fieldRelative = false;
            RobotState.getInstance().setRobotState(RobotStates.CORAL_READY);

            return AutoBuilder.buildAuto(auto);
        }
    }
}
