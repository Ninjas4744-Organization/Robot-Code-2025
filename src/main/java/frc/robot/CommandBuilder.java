package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public static Command setReefLevel(int level){
        return Commands.runOnce(() -> {
            if(RobotState.getInstance().getRobotState() != RobotStates.GO_RIGHT_REEF
                    && RobotState.getInstance().getRobotState() != RobotStates.GO_LEFT_REEF
                    && RobotState.getInstance().getRobotState() != RobotStates.AT_SIDE_REEF
                    && RobotState.getInstance().getRobotState() != RobotStates.OUTTAKE_READY
                    && RobotState.getInstance().getRobotState() != RobotStates.OUTTAKE)
                RobotState.getInstance().setReefLevel(level);
        });
    }

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
                () -> RobotState.getInstance().getRobotPose(), // Robot pose supplier

                pose -> {
                    RobotState.getInstance().setRobotPose(pose);
                    RobotState.getInstance().resetGyro(pose.getRotation());
                }, // Method to reset odometry (will be called if your auto has a starting pose)

                () -> SwerveIO.getInstance().getChassisSpeeds(false), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

                drive -> SwerveController.getInstance().Demand.velocity = drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

                SwerveConstants.kAutonomyConfig, //Autonomy config
                SwerveConstants.kSwerveControllerConstants.robotConfig, //Robot config

                () -> false, // Boolean supplier that mirrors path if red alliance

                SwerveSubsystem.getInstance() // Reference to swerve subsystem to set requirements
            );

            registerCommands();
        }

        /** Registers all auto commands to pathplanner */
        private static void registerCommands() {
            NamedCommands.registerCommand("Intake", intake());
            NamedCommands.registerCommand("Wait Outtake", waitOuttake());
            NamedCommands.registerCommand("Print 1", Commands.print("11111111111111"));

            /* Named commands for object detection */
//          NamedCommands.registerCommand("L1", L1());
//          NamedCommands.registerCommand("L2", L2());
//          NamedCommands.registerCommand("L3", L3());
//          NamedCommands.registerCommand("L4", L4());

            /* Named commands without object detection */
            NamedCommands.registerCommand("Right", Right(4));
            NamedCommands.registerCommand("Left", Left(4));
        }

        public static Command intake() {
            return Commands.sequence(
                    CommandBuilder.changeRobotState(RobotStates.INTAKE),
                    Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.CORAL_READY)
            );
        }

        public static Command waitOuttake() {
            return Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.CORAL_SEARCH)
                    .andThen(Commands.runOnce(() -> SwerveController.getInstance().Demand.fieldRelative = false));
        }

          /* Commands for object detection */
//        public static Command L1() {
//            return CommandBuilder.Teleop.changeRobotState(RobotStates.L1);
//        }
//
//        public static Command L2() {
//            return CommandBuilder.Teleop.changeRobotState(RobotStates.L2);
//        }
//
//        public static Command L3() {
//            return CommandBuilder.Teleop.changeRobotState(RobotStates.L3);
//        }
//
//        public static Command L4() {
//            return CommandBuilder.Teleop.changeRobotState(RobotStates.L4);
//        }

        /* Commands without object detection */
        public static Command Right(int level) {
            return Commands.sequence(
                    Commands.runOnce(() -> RobotState.getInstance().setReefLevel(level)),
                    CommandBuilder.changeRobotState(RobotStates.GO_RIGHT_REEF)
            );
        }

        public static Command Left(int level) {
            return Commands.sequence(
                    Commands.runOnce(() -> RobotState.getInstance().setReefLevel(level)),
                    CommandBuilder.changeRobotState(RobotStates.GO_LEFT_REEF)
            );
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
