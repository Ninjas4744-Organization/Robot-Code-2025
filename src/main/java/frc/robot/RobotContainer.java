package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import frc.robot.StateMachine.StateMachine;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
    private final CommandPS5Controller _driverJoystick;
	private final CommandPS5Controller _operatorJoystick;

    private boolean isSwerveLookAt = false;

    public RobotContainer() {
        SwerveIO.setConstants(SwerveConstants.kSwerveConstants);
        RobotStateWithSwerve.setInstance(new RobotState(), SwerveConstants.kSwerveConstants.kinematics, SwerveConstants.kInvertGyro, VisionConstants::calculateFOM, 2);
        SwerveSubsystem.getInstance();
        StateMachineIO.setInstance(new StateMachine());
        VisionIO.setConstants(VisionConstants.kVisionConstants);
        FieldConstants.getFieldLayout();

        CommandBuilder.Auto.configureAutoBuilder();
        Shuffleboard.getTab("Competition").addString("Robot State", () -> RobotState.getInstance().getRobotState().toString());

        _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
        _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
        configureBindings();
    }

    private void configureBindings() {
        StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.povLeft());

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

        _driverJoystick.R1().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));

        _driverJoystick.L1().onTrue(CommandBuilder.Teleop.resetGyro(false));
        _driverJoystick.L2().onTrue(CommandBuilder.Teleop.resetGyro(true));
    }

    private void configureOperatorBindings() {
        _driverJoystick.povDown().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(RobotStates.RESET);
            if(!RobotState.isSimulated())
			    ((Swerve)SwerveIO.getInstance()).resetModulesToAbsolute();
        })));

        _driverJoystick.cross().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.L1)));
//        _driverJoystick.square().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.GO_RIGHT_REEF)));
    }

    private void configureTestBindings() {

    }

    public void periodic() {
        for (VisionOutput estimation : VisionIO.getInstance().getVisionEstimations())
            if (estimation.robotPose != null)
                RobotState.getInstance().updateRobotPose(estimation);
    }

    public void resetSubsystems() {
        RobotState.getInstance().setRobotState(RobotStates.RESET);
        CommandBuilder.Teleop.resetGyro(false).schedule();
    }
}
