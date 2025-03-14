package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import frc.robot.StateMachine.StateMachine;
import frc.robot.Subsystems.*;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private final CommandPS5Controller _driverJoystick;
	private final CommandPS5Controller _operatorJoystick;

    private boolean isSwerveLookAt = false;

    public RobotContainer() {
        SwerveIO.setConstants(SwerveConstants.kSwerveConstants);
        RobotStateWithSwerve.setInstance(new RobotState(),
                SwerveConstants.kSwerveConstants.kinematics,
                SwerveConstants.kInvertGyro,
                VisionConstants::calculateFOM,
                Constants.kPigeonID);

        SwerveSubsystem.createInstance(false);
        Elevator.createInstance(false);
        Leds.createInstance(false);
        Outtake.createInstance(false);
        OuttakeAngle.createInstance(false);
        Sushi.createInstance(false);

        StateMachineIO.setInstance(new StateMachine(false));
        VisionIO.setConstants(VisionConstants.kVisionConstants);
        LimelightVision.init();
        FieldConstants.getFieldLayout();

        CommandBuilder.Auto.configureAutoBuilder();

        LiveWindow.disableAllTelemetry();

        _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
        _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
        configureBindings();
    }

    private void configureBindings() {
        StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.povLeft());

        //Auto Intake, NOTE: may or may not crash auto
//        new Trigger(FieldConstants::nearCoralStation).onTrue(CommandBuilder.changeRobotState(RobotStates.INTAKE));
//        new Trigger(() -> !FieldConstants.nearCoralStation() && RobotState.getInstance().getRobotState() == RobotStates.INTAKE).onTrue(CommandBuilder.changeRobotState(RobotStates.IDLE));

        configureTestBindings();
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        SwerveSubsystem.getInstance()
          .setDefaultCommand(CommandBuilder.Teleop.swerveDrive(
            () -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
            () -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
            () -> isSwerveLookAt,
            () -> false));

        _driverJoystick.cross().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt)));

        _driverJoystick.L1().onTrue(CommandBuilder.resetGyro(false));
        _driverJoystick.R1().onTrue(CommandBuilder.resetGyro(true));

        _driverJoystick.L2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.GO_LEFT_REEF)));
        _driverJoystick.R2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.GO_RIGHT_REEF)));
    }

    private void configureOperatorBindings() {
        /* Option 1 */
//        _driverJoystick.povDown().onTrue (CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(1))));
//        _driverJoystick.povRight().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(2))));
//        _driverJoystick.povUp().onTrue   (CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(3))));
//        _driverJoystick.povLeft().onTrue (CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(4))));
//        _driverJoystick.L1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.GO_LEFT_REEF)));
//        _driverJoystick.R1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.GO_RIGHT_REEF)));
//
//        _driverJoystick.cross().onTrue(CommandBuilder.changeRobotState(RobotStates.INTAKE));
//        _driverJoystick.circle().onTrue(CommandBuilder.changeRobotState(RobotStates.CLOSE));
//        _driverJoystick.square().onTrue(CommandBuilder.changeRobotState(RobotStates.REMOVE_ALGAE));
//
//        _driverJoystick.triangle().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
//            StateMachine.getInstance().changeRobotState(RobotStates.RESET);
//            if(!RobotState.isSimulated())
//                ((Swerve)SwerveIO.getInstance()).resetModulesToAbsolute();
//        })));

        /* Option 2 */
        _operatorJoystick.povDown().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(RobotStates.RESET);
            if(!RobotState.isSimulated())
                ((Swerve)SwerveIO.getInstance()).resetModulesToAbsolute();
        })));

        _operatorJoystick.L1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeReefLevel(false)));
        _operatorJoystick.R1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeReefLevel(true)));

        _operatorJoystick.cross().onTrue(CommandBuilder.changeRobotState(RobotStates.INTAKE));
        _operatorJoystick.circle().onTrue(CommandBuilder.changeRobotState(RobotStates.CLOSE));
        _operatorJoystick.triangle().onTrue(Commands.either(
                CommandBuilder.changeRobotState(RobotStates.AT_SIDE_REEF),
                Commands.runOnce(() -> RobotState.getInstance().setRobotState(RobotStates.OUTTAKE_READY)),
                () -> RobotState.getInstance().getRobotState() != RobotStates.AT_SIDE_REEF
        ));
        _operatorJoystick.square().onTrue(CommandBuilder.changeRobotState(RobotStates.REMOVE_ALGAE));
    }

    private void configureTestBindings() {
        _driverJoystick.square().whileTrue(CommandBuilder.Teleop.runIfTestMode(Outtake.getInstance().runMotor(0.5)));
        _driverJoystick.circle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Outtake.getInstance().runMotor(-0.5)));
        _driverJoystick.triangle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(0.15)));
        _driverJoystick.cross().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(-0.15)));
        _driverJoystick.povUp().whileTrue(CommandBuilder.Teleop.runIfTestMode(OuttakeAngle.getInstance().runMotor(0.05)));
        _driverJoystick.povDown().whileTrue(CommandBuilder.Teleop.runIfTestMode(OuttakeAngle.getInstance().runMotor(-0.05)));
    }

    public void periodic() {
        for (VisionOutput estimation : VisionIO.getInstance().getVisionEstimations())
            if (estimation.robotPose != null)
                RobotState.getInstance().updateRobotPose(estimation);
//        RobotState.getInstance().updateRobotPose(LimelightVision.getVisionEstimation());
        Logger.recordOutput("Right", VisionIO.getInstance().getVisionEstimations()[0].robotPose);
        Logger.recordOutput("Left", VisionIO.getInstance().getVisionEstimations()[1].robotPose);

        SmartDashboard.putString("Competition/Robot State", RobotState.getInstance().getRobotState().toString());
        SmartDashboard.putNumber("Competition/Reef Level", RobotState.getInstance().getReefLevel());
        SmartDashboard.putBoolean("Competition/Beam Breaker", RobotState.getInstance().isCoralInRobot());
    }

    public void resetSubsystems() {
        RobotState.getInstance().setRobotState(RobotStates.RESET);
        CommandBuilder.resetGyro(false).schedule();
    }
}
