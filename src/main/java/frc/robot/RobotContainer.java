package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateIO;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import frc.robot.StateMachine.StateMachine;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

public class RobotContainer {
	private final CommandPS5Controller _driverJoystick;
//	private final CommandPS5Controller _operatorJoystick;

	private boolean isSwerveLookAt = false;

	public RobotContainer() {
		RobotStateIO.setInstance(new RobotState(), Robot.instance);
		Shooter.getInstance();
		ShooterAngle.getInstance();
		StateMachineIO.setInstance(new StateMachine());
		Indexer.getInstance();
		FieldConstants.getFieldLayout();

		VisionIO.setConstants(VisionConstants.kVisionConstants);
		RobotState.getInstance().initPoseEstimator();
		CommandBuilder.Auto.configureAutoBuilder();

		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
//		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		Shuffleboard.getTab("Competition")
				.addString("Robot State", () -> RobotState.getInstance().getRobotState().toString());

		configureBindings();
	}

	private void configureBindings() {
		StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.povLeft());

		configureTestBindings();
		configureDriverBindings();
		configureOperatorBindings();
	}

	private void configureDriverBindings() {
//		SwerveIO.getInstance()
//				.setDefaultCommand(CommandBuilder.Teleop.swerveDrive(
//						() -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
//						() -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
//						() -> isSwerveLookAt,
//						() -> false));

		_driverJoystick.R1().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));
		_driverJoystick.R2().onTrue(CommandBuilder.Teleop.changeRobotState(RobotStates.SHOOT));

		_driverJoystick.L1().onTrue(CommandBuilder.Teleop.resetGyro(false));
		_driverJoystick.L2().onTrue(CommandBuilder.Teleop.resetGyro(true));
	}

	private void configureOperatorBindings() {
		_driverJoystick.cross().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.INTAKE)));

		_driverJoystick.triangle().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
			if (RobotState.getInstance().getRobotPose().getX() <= 5)
				StateMachine.getInstance().changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE);
			else StateMachine.getInstance().changeRobotState(RobotStates.DELIVERY);
		})));

		_driverJoystick.square().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.DRIVE_TO_AMP)));

		_driverJoystick.circle().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.CLOSE)));

		_driverJoystick.R2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(
						() -> {
							StateMachine.getInstance().changeRobotState(RobotStates.SHOOT);
							StateMachine.getInstance().changeRobotState(RobotStates.OUTTAKE);
						},
						StateMachine.getInstance())));

		_driverJoystick.povUp().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.OOGA_BOOGA)));

		_driverJoystick.povDown().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
			StateMachine.getInstance().changeRobotState(RobotStates.RESET);
//			((Swerve) (SwerveIO.getInstance())).resetModulesToAbsolute();
		})));
	}

	private void configureTestBindings() {
		_driverJoystick.triangle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Indexer.getInstance().runMotor(1)));
		_driverJoystick.cross().whileTrue(CommandBuilder.Teleop.runIfTestMode(Indexer.getInstance().runMotor(-1)));

		_driverJoystick.povDown().whileTrue(CommandBuilder.Teleop.runIfTestMode(ShooterAngle.getInstance().runMotor(-0.5)));
		_driverJoystick.povUp().whileTrue(CommandBuilder.Teleop.runIfTestMode(ShooterAngle.getInstance().runMotor(0.5)));

		_driverJoystick.square().whileTrue(CommandBuilder.Teleop.runIfTestMode(Shooter.getInstance().runMotor(1)));
	}

	public void periodic() {
		for (VisionOutput estimation : VisionIO.getInstance().getVisionEstimations())
			if (estimation.robotPose != null)
				RobotState.getInstance().updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.getInstance().setRobotState(RobotStates.RESET);
		Shooter.getInstance().resetSubsystem();
		Indexer.getInstance().resetSubsystem();
		ShooterAngle.getInstance().resetSubsystem();

//		SwerveIO.getInstance().setState(SwerveDemand.SwerveState.DEFAULT);
//		SwerveIO.getInstance().drive(new ChassisSpeeds(), false);
		CommandBuilder.Teleop.resetGyro(false).schedule();
	}
}
