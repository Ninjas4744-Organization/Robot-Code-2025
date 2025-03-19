package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import frc.robot.StateMachine.StateMachine;
import frc.robot.Subsystems.*;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
        Climber.createInstance(false);
        HopperAngle.createInstance(false);

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
//        CommandBuilder.Teleop.swerveDrive(
//            () -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
//            () -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
//            () -> isSwerveLookAt,
//            () -> false);

        _driverJoystick.cross().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt)));

        _driverJoystick.L1().onTrue(CommandBuilder.resetGyro(false));
        _driverJoystick.R1().onTrue(CommandBuilder.resetGyro(true));

        _driverJoystick.L2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.sequence(
                Commands.runOnce(() -> RobotState.getInstance().setReefRight(false)),
                CommandBuilder.changeRobotState(RobotStates.GO_REEF)))
        );

        _driverJoystick.R2().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.sequence(
                Commands.runOnce(() -> RobotState.getInstance().setReefRight(true)),
                CommandBuilder.changeRobotState(RobotStates.GO_REEF)))
        );
    }

    private void configureOperatorBindings() {
        _operatorJoystick.povDown().onTrue(CommandBuilder.Teleop.runIfNotTestMode(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(RobotStates.RESET);
            if(!RobotState.isSimulated())
                ((Swerve)SwerveIO.getInstance()).resetModulesToAbsolute();
        })));

        _operatorJoystick.L1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeReefLevel(false)));
        _operatorJoystick.R1().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeReefLevel(true)));

        _operatorJoystick.cross().onTrue   (CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.INTAKE)));
        _operatorJoystick.circle().onTrue  (CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.CLOSE)));
        _operatorJoystick.triangle().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.AT_REEF)));
        _operatorJoystick.square().onTrue  (CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.switchAlgaeState()));

        _operatorJoystick.povRight().onTrue(CommandBuilder.Teleop.runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.OUTTAKE)));

        _operatorJoystick.povUp().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(RobotStates.CLIMB2);
            StateMachine.getInstance().changeRobotState(RobotStates.CLIMB1);
        }));
    }

    private void configureTestBindings() {
        _operatorJoystick.triangle().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(0.15)));
        _operatorJoystick.cross().whileTrue(CommandBuilder.Teleop.runIfTestMode(Elevator.getInstance().runMotor(-0.15)));
        _operatorJoystick.povUp().whileTrue(CommandBuilder.Teleop.runIfTestMode(Climber.getInstance().runMotor(-1)));
        _operatorJoystick.povDown().whileTrue(CommandBuilder.Teleop.runIfTestMode(Climber.getInstance().runMotor(1)));
        _operatorJoystick.povRight().whileTrue(CommandBuilder.Teleop.runIfTestMode(HopperAngle.getInstance().runMotor(0.1)));
        _operatorJoystick.povLeft().whileTrue(CommandBuilder.Teleop.runIfTestMode(HopperAngle.getInstance().runMotor(-0.1)));
    }


    private static PIDController _lookAtCenterReefPID = new PIDController(0.01, 0, 0);
    static {
        _lookAtCenterReefPID.enableContinuousInput(-180, 180);
    }

    private void swerveDrive(
            Translation2d translation,
            Translation2d rotation,
            boolean isLookAt,
            boolean isBayblade){
        double lx = -MathUtil.applyDeadband(translation.getX(), SwerveConstants.kJoystickDeadband);
        double ly = -MathUtil.applyDeadband(translation.getY(), SwerveConstants.kJoystickDeadband);
        double rx = -MathUtil.applyDeadband(rotation.getX(), SwerveConstants.kJoystickDeadband);
        double ry = -MathUtil.applyDeadband(rotation.getY(), SwerveConstants.kJoystickDeadband);

        double finalRotation = rx * SwerveConstants.kDriverRotationSpeedFactor;

        if (isLookAt)
            finalRotation = _lookAtCenterReefPID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), RobotState.getInstance().getTransform(new Pose2d(4.49, 4.03, Rotation2d.kZero)).getTranslation().getAngle().getDegrees());
//                            finalRotation = SwerveController.getInstance().lookAtTarget(new Pose2d(4.49, 4.03, Rotation2d.kZero), Rotation2d.kZero);
//                            finalRotation = SwerveController.getInstance().lookAt(new Translation2d(ry, rx), 45);

        if (isBayblade)
            finalRotation = 1;

        SwerveController.getInstance().setControl(
                SwerveController.getInstance().fromPercent(new ChassisSpeeds(
                        ly * SwerveConstants.kDriverSpeedFactor,
                        lx * SwerveConstants.kDriverSpeedFactor,
                        finalRotation)),
                SwerveConstants.kDriverFieldRelative, "Driver"
        );
    }

    public void periodic() {
        swerveDrive(
                new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
                new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
                isSwerveLookAt,
                false);

        for (VisionOutput estimation : VisionIO.getInstance().getVisionEstimations())
                RobotState.getInstance().updateRobotPose(estimation);
//        RobotState.getInstance().updateRobotPose(LimelightVision.getVisionEstimation());
//        Logger.recordOutput("Right", VisionIO.getInstance().getVisionEstimations()[0].robotPose);
//        Logger.recordOutput("Left", VisionIO.getInstance().getVisionEstimations()[1].robotPose);

        SmartDashboard.putString("Competition/Robot State", RobotState.getInstance().getRobotState().toString());
        SmartDashboard.putNumber("Competition/Reef Level", RobotState.getInstance().getReefLevel());
        SmartDashboard.putBoolean("Competition/Beam Breaker", RobotState.getInstance().isCoralInRobot());
    }

    public void resetSubsystems() {
        StateMachine.getInstance().changeRobotState(RobotStates.RESET);
        CommandBuilder.resetGyro(false).schedule();
    }
}
