package frc.robot;

import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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

import static edu.wpi.first.units.Units.Seconds;

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

    public static Command changeReefLevel(boolean up){
        return Commands.runOnce(() -> setReefLevel(MathUtil.clamp(RobotState.getInstance().getReefLevel() + (up ? 1 : -1), 1, 4)).schedule());
    }

    public static Command setReefLevel(int level){
        return Commands.runOnce(() -> {
            if(RobotState.getInstance().getRobotState() != RobotStates.OUTTAKE)
                RobotState.getInstance().setReefLevel(level);
        });
    }

    public static Command switchAlgaeState(){
        return Commands.runOnce(() -> {
           RobotState.getInstance().setAlgae(!RobotState.getInstance().getAlgae());
        });
    }

    public static class Teleop{
        private static PIDController _lookAtCenterReefPID = new PIDController(0.01, 0, 0);
        static {
            _lookAtCenterReefPID.enableContinuousInput(-180, 180);
        }

        public static Command swerveDrive(
                Supplier<Translation2d> translation,
                Supplier<Translation2d> rotation,
                BooleanSupplier isLookAt,
                BooleanSupplier isBayblade) {
            return Commands.run(
                    () -> {
                        double lx = -MathUtil.applyDeadband(translation.get().getX(), SwerveConstants.kJoystickDeadband);
                        double ly = -MathUtil.applyDeadband(translation.get().getY(), SwerveConstants.kJoystickDeadband);
                        double rx = -MathUtil.applyDeadband(rotation.get().getX(), SwerveConstants.kJoystickDeadband);
                        double ry = -MathUtil.applyDeadband(rotation.get().getY(), SwerveConstants.kJoystickDeadband);

                        double finalRotation = rx * SwerveConstants.kDriverRotationSpeedFactor;

                        if (isLookAt.getAsBoolean())
                            finalRotation = _lookAtCenterReefPID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), RobotState.getInstance().getTransform(new Pose2d(4.49, 4.03, Rotation2d.kZero)).getTranslation().getAngle().getDegrees());
//                            finalRotation = SwerveController.getInstance().lookAtTarget(new Pose2d(4.49, 4.03, Rotation2d.kZero), Rotation2d.kZero);
//                            finalRotation = SwerveController.getInstance().lookAt(new Translation2d(ry, rx), 45);

                        if (isBayblade.getAsBoolean())
                            finalRotation = 1;

                        SwerveController.getInstance().setControl(
                            SwerveController.getInstance().fromPercent(new ChassisSpeeds(
                                ly * SwerveConstants.kDriverSpeedFactor,
                                lx * SwerveConstants.kDriverSpeedFactor,
                                finalRotation)),
                            SwerveConstants.kDriverFieldRelative, "Driver"
                        );
                    });
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

                drive -> SwerveController.getInstance().setControl(drive, false, "Auto"), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

                SwerveConstants.kAutonomyConfig, //Autonomy config
                SwerveConstants.kSwerveControllerConstants.robotConfig, //Robot config

                () -> false // Boolean supplier that mirrors path if red alliance

                 //SwerveSubsystem.getInstance() // Reference to swerve subsystem to set requirements
            );

            registerCommands();
        }

        /** Registers all auto commands to pathplanner */
        private static void registerCommands() {
            NamedCommands.registerCommand("Wait Reset", waitReset());
            NamedCommands.registerCommand("Intake", intake());
            NamedCommands.registerCommand("Wait Outtake", waitOuttake());
            NamedCommands.registerCommand("Right 4", Right(4));
            NamedCommands.registerCommand("Left 4", Left(4));
            NamedCommands.registerCommand("Right 3", Right(3));
            NamedCommands.registerCommand("Left 3", Left(3));
            NamedCommands.registerCommand("Print 1", Commands.print("1111111111111"));
        }

        private static Command waitReset(){
            return Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.CORAL_READY);
        }

        private static Command intake() {
            return Commands.sequence(
                    CommandBuilder.changeRobotState(RobotStates.INTAKE),
                    Commands.run(() -> SwerveController.getInstance().setControl(new ChassisSpeeds(), false, "Auto"))
                            .until(() -> RobotState.getInstance().isCoralInRobot())
            );
        }

        private static Command waitOuttake() {
            return Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.CLOSE).andThen(Commands.runOnce(() -> SwerveController.getInstance().setState("Auto")));
        }

        private static Command Right(int level) {
            return Commands.sequence(
                    Commands.runOnce(() -> RobotState.getInstance().setReefLevel(level)),
                    Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.CORAL_READY),
                    Commands.waitTime(Seconds.of(0.02)),
                    Commands.runOnce(() -> RobotState.getInstance().setReefRight(true)),
                    Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(RobotStates.GO_REEF))
            );
        }

        private static Command Left(int level) {
            return Commands.sequence(
                    Commands.runOnce(() -> RobotState.getInstance().setReefLevel(level)),
                    Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.CORAL_READY),
                    Commands.waitTime(Seconds.of(0.02)),
                    Commands.runOnce(() -> RobotState.getInstance().setReefRight(false)),
                    Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(RobotStates.GO_REEF))
            );
        }
    }
}
