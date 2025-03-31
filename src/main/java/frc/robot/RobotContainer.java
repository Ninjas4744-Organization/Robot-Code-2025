package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.Swerve;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

import static frc.robot.CommandBuilder.Teleop.runIfNotTestMode;
import static frc.robot.CommandBuilder.Teleop.runIfTestMode;

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
        FieldConstants.getFieldLayout();

        CommandBuilder.Auto.configureAutoBuilder();

        LiveWindow.disableAllTelemetry();
        CommandLogger.initialize();

//        CameraServer.startAutomaticCapture();

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

        _driverJoystick.cross().onTrue(runIfNotTestMode(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt)));

        _driverJoystick.L1().onTrue(CommandBuilder.resetGyro(false));
        _driverJoystick.R1().onTrue(CommandBuilder.resetGyro(true));

        _driverJoystick.L2().onTrue(runIfNotTestMode(Commands.sequence(
                Commands.runOnce(() -> RobotState.getInstance().setReefRight(false)),
                CommandBuilder.changeRobotState(RobotStates.GO_REEF)))
        );

        _driverJoystick.R2().onTrue(runIfNotTestMode(Commands.sequence(
                Commands.runOnce(() -> RobotState.getInstance().setReefRight(true)),
                CommandBuilder.changeRobotState(RobotStates.GO_REEF)))
        );
    }

    private void configureOperatorBindings() {
        _operatorJoystick.povDown().onTrue(runIfNotTestMode(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(RobotStates.RESET);
            if(!RobotState.isSimulated())
                ((Swerve)SwerveIO.getInstance()).resetModulesToAbsolute();
        })));

        _operatorJoystick.L1().onTrue(runIfNotTestMode(CommandBuilder.changeReefLevel(false)));
        _operatorJoystick.R1().onTrue(runIfNotTestMode(CommandBuilder.changeReefLevel(true)));

        _operatorJoystick.cross().onTrue   (runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.INTAKE)));
        _operatorJoystick.circle().onTrue  (runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.CLOSE)));
        _operatorJoystick.triangle().onTrue(runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.AT_REEF)));
        _operatorJoystick.square().onTrue  (runIfNotTestMode(CommandBuilder.switchAlgaeState()));

        _operatorJoystick.povRight().onTrue(runIfNotTestMode(CommandBuilder.changeRobotState(RobotStates.OUTTAKE)));

        _operatorJoystick.options().onTrue(runIfNotTestMode(Commands.runOnce(() -> {
//            StateMachine.getInstance().changeRobotState(RobotStates.CLIMB2);
            StateMachine.getInstance().changeRobotState(RobotStates.CLIMB1);
        })));

        _operatorJoystick.options().whileTrue(runIfNotTestMode(Commands.either(
           Climber.getInstance().runMotor(1),
           Commands.none(),
           () -> RobotState.getInstance().getRobotState() == RobotStates.CLIMBED1
        )));

        _operatorJoystick.create().whileTrue(runIfNotTestMode(Commands.either(
                HopperAngle.getInstance().runMotor(0.1),
                Commands.none(),
                () -> RobotState.getInstance().getRobotState() == RobotStates.CLIMBED1
        )));
    }

    private void configureTestBindings() {
        _operatorJoystick.triangle().whileTrue(runIfTestMode(Elevator.getInstance().runMotor(0.15)));
        _operatorJoystick.cross().whileTrue(runIfTestMode(Elevator.getInstance().runMotor(-0.15)));
        _operatorJoystick.povUp().whileTrue(runIfTestMode(Climber.getInstance().runMotor(-1)));
        _operatorJoystick.povDown().whileTrue(runIfTestMode(Climber.getInstance().runMotor(1)));
        _operatorJoystick.povRight().whileTrue(runIfTestMode(HopperAngle.getInstance().runMotor(0.1)));
        _operatorJoystick.povLeft().whileTrue(runIfTestMode(HopperAngle.getInstance().runMotor(-0.1)));
    }

    private static final ProfiledPIDController _lookAtCenterReefPID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(460, 1000));
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

        double finalRotation = rx * SwerveConstants.kDriverRotationSpeedFactor * SwerveConstants.kSwerveConstants.maxAngularVelocity;

        if (isLookAt){
            finalRotation = _lookAtCenterReefPID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), RobotState.getInstance().getTransform(new Pose2d(4.49, 4.03, Rotation2d.kZero)).getTranslation().getAngle().getDegrees());
//                            finalRotation = SwerveController.getInstance().lookAtTarget(new Pose2d(4.49, 4.03, Rotation2d.kZero), Rotation2d.kZero);
//                            finalRotation = SwerveController.getInstance().lookAt(new Translation2d(ry, rx), 45);
//            finalRotation = _lookAtCenterReefPID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), FieldConstants.getClosestReefTag().getRotation().rotateBy(Rotation2d.k180deg).getDegrees());
        }

        if (isBayblade)
            finalRotation = 1;

        ChassisSpeeds fromPercent = SwerveController.getInstance().fromPercent(new ChassisSpeeds(
                ly * SwerveConstants.kDriverSpeedFactor,
                lx * SwerveConstants.kDriverSpeedFactor
                , 0));

        SwerveController.getInstance().setControl(new ChassisSpeeds(
                fromPercent.vxMetersPerSecond,
                fromPercent.vyMetersPerSecond,
                finalRotation),
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

        Logger.recordOutput("Other/Right Camera", VisionIO.getInstance().getVisionEstimations()[0].robotPose);
        Logger.recordOutput("Other/Left Camera", VisionIO.getInstance().getVisionEstimations()[1].robotPose);
//        Logger.recordOutput("Other/Back Camera", VisionIO.getInstance().getVisionEstimations()[2].robotPose);

        Logger.recordOutput("Beam Breaker", RobotState.getInstance().isCoralInRobot());
    }

    public void resetSubsystems() {
        StateMachine.getInstance().changeRobotState(RobotStates.RESET);
        CommandBuilder.resetGyro(false).schedule();
    }
}
