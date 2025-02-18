package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
        Leds.createInstance(true);
        Outtake.createInstance(true);
        OuttakeAngle.createInstance(true);
        Sushi.createInstance(false);

        StateMachineIO.setInstance(new StateMachine(false));
        VisionIO.setConstants(VisionConstants.kVisionConstants);
        FieldConstants.getFieldLayout();

        CommandBuilder.Auto.configureAutoBuilder();
        Shuffleboard.getTab("Competition").addString("Robot State", () -> RobotState.getInstance().getRobotState().toString());
        Shuffleboard.getTab("Competition").addString("Coral Detection", () -> CoralObjectDetection.getCoralDetection().toString());
        Shuffleboard.getTab("Competition").addInteger("Reef Level", () -> RobotState.getInstance().getReefLevel());

        LiveWindow.disableAllTelemetry();

        _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
        _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
        configureBindings();
    }

    private void configureBindings() {
        StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.povLeft());

        //Auto Intake
        new Trigger(FieldConstants::nearCoralStation).onTrue(CommandBuilder.Teleop.changeRobotState(RobotStates.INTAKE));
        new Trigger(() -> !FieldConstants.nearCoralStation() && RobotState.getInstance().getRobotState() == RobotStates.INTAKE).onTrue(CommandBuilder.Teleop.changeRobotState(RobotStates.IDLE));

        configureTestBindings();
        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        SwerveSubsystem.getInstance()
          .setDefaultCommand(CommandBuilder.swerveDrive(
            () -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
            () -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
            () -> isSwerveLookAt,
            () -> false));

        _driverJoystick.R1().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));

        _driverJoystick.L1().onTrue(CommandBuilder.resetGyro(false));
        _driverJoystick.L2().onTrue(CommandBuilder.resetGyro(true));
    }

    private void configureOperatorBindings() {
        _driverJoystick.povDown().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(RobotStates.RESET);
            if(!RobotState.isSimulated())
			    ((Swerve)SwerveIO.getInstance()).resetModulesToAbsolute();
        })));

        /* Object Detection */
//        _driverJoystick.cross().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.L1)));
//        _driverJoystick.circle().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.CLOSE)));
//        _driverJoystick.square().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.INTAKE)));

//        _driverJoystick.cross().onTrue   (CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(1))));
//        _driverJoystick.circle().onTrue  (CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(2))));
//        _driverJoystick.triangle().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(3))));
//        _driverJoystick.square().onTrue  (CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(4))));
//        _driverJoystick.L1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.GO_LEFT_REEF)));
//        _driverJoystick.R1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.Teleop.changeRobotState(RobotStates.GO_RIGHT_REEF)));

        /* Test */
        _driverJoystick.cross().onTrue(Commands.runOnce(() -> RobotState.getInstance().setRobotState(RobotStates.AT_SIDE_REEF)));
        _driverJoystick.circle().onTrue(Commands.runOnce(() -> RobotState.getInstance().setRobotState(RobotStates.CLOSE)));
        _driverJoystick.triangle().onTrue(Commands.runOnce(() -> RobotState.getInstance().setReefLevel(Math.max(1, (RobotState.getInstance().getReefLevel() + 1) % 5))));
        _driverJoystick.square().onTrue(Commands.runOnce(() -> RobotState.getInstance().setRobotState(RobotStates.INTAKE)));
    }

    private void configureTestBindings() {
//        _driverJoystick.square().whileTrue(CommandBuilder.Teleop.runIfTestMode(Outtake.getInstance().runMotor(0.5)));
//        _driverJoystick.circle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Outtake.getInstance().runMotor(-0.5)));
        _driverJoystick.triangle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(0.5)));
        _driverJoystick.cross().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(-0.5)));
    }

    public void periodic() {
//        for (VisionOutput estimation : VisionIO.getInstance().getVisionEstimations())
//            if (estimation.robotPose != null)
//                RobotState.getInstance().updateRobotPose(estimation);
    }

    public void resetSubsystems() {
        RobotState.getInstance().setRobotState(RobotStates.RESET);
        CommandBuilder.resetGyro(false).schedule();
    }
}
