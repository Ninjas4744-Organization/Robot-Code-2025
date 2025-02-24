package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
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
        Leds.createInstance(false);
        Outtake.createInstance(false);
        OuttakeAngle.createInstance(false);
        Sushi.createInstance(false);

        StateMachineIO.setInstance(new StateMachine(false));
        VisionIO.setConstants(VisionConstants.kVisionConstants);
        FieldConstants.getFieldLayout();

        CommandBuilder.Auto.configureAutoBuilder();

        LiveWindow.disableAllTelemetry();

        _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
        _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
        configureBindings();
    }

    private void configureBindings() {
        StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.povLeft());

        //Auto Intake
        new Trigger(FieldConstants::nearCoralStation).onTrue(CommandBuilder.changeRobotState(RobotStates.INTAKE));
        new Trigger(() -> !FieldConstants.nearCoralStation() && RobotState.getInstance().getRobotState() == RobotStates.INTAKE).onTrue(CommandBuilder.changeRobotState(RobotStates.IDLE));

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

//        _driverJoystick.R1().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));

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
        _operatorJoystick.povUp().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));

        _operatorJoystick.L1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeReefLevel(false)));
        _operatorJoystick.R1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeReefLevel(true)));
        _operatorJoystick.L2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.GO_LEFT_REEF)));
        _operatorJoystick.R2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.GO_RIGHT_REEF)));

        _operatorJoystick.cross().onTrue(CommandBuilder.changeRobotState(RobotStates.INTAKE));
        _operatorJoystick.circle().onTrue(CommandBuilder.changeRobotState(RobotStates.CLOSE));
        _operatorJoystick.square().onTrue(Commands.sequence(
            Commands.runOnce(() -> RobotState.getInstance().setAlgaeLevel(Math.max(((RobotState.getInstance().getAlgaeLevel() + 1) % 3), 1))),
            CommandBuilder.changeRobotState(RobotStates.REMOVE_ALGAE)
        ));
    }

    private void configureTestBindings() {
        _driverJoystick.square().whileTrue(CommandBuilder.Teleop.runIfTestMode(Outtake.getInstance().runMotor(0.5)));
        _driverJoystick.circle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Outtake.getInstance().runMotor(-0.5)));
        _driverJoystick.triangle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(0.15)));
        _driverJoystick.cross().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(-0.15)));
        _driverJoystick.povUp().whileTrue(CommandBuilder.Teleop.runIfTestMode(OuttakeAngle.getInstance().runMotor(0.05)));
        _driverJoystick.povDown().whileTrue(CommandBuilder.Teleop.runIfTestMode(OuttakeAngle.getInstance().runMotor(-0.05)));
    }

    public static <T> void logToShuffleboard(String key, T value){
        NetworkTableInstance.getDefault().getTable("Competition").getEntry(key).setValue(value);
    }

    public static NetworkTableValue getFromShuffleboard(String key){
        return NetworkTableInstance.getDefault().getTable("Competition").getEntry(key).getValue();
    }

    public void periodic() {
        for (VisionOutput estimation : VisionIO.getInstance().getVisionEstimations())
            if (estimation.robotPose != null)
                RobotState.getInstance().updateRobotPose(estimation);

        logToShuffleboard("Robot State", RobotState.getInstance().getRobotState().toString());
        logToShuffleboard("Reef Level", RobotState.getInstance().getReefLevel());
        logToShuffleboard("Beam Breaker", RobotState.getInstance().isCoralInRobot());
    }

    public void resetSubsystems() {
        RobotState.getInstance().setRobotState(RobotStates.RESET);
        CommandBuilder.resetGyro(false).schedule();
    }
}
