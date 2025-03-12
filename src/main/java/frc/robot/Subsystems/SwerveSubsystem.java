package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class SwerveSubsystem extends StateMachineSubsystem<RobotStates> {
    private static SwerveSubsystem _instance;

    public static SwerveSubsystem getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new SwerveSubsystem(paused);
    }

    private Pose2d _currentReefTarget = new Pose2d();
    private final ProfiledPIDController _xPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(4.5, 3));
    private final ProfiledPIDController _yPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(4.5, 3));
    private final ProfiledPIDController _0PID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(460, 1000));
    private double _outtakeExtraMove = 0;

    private SwerveSubsystem(boolean paused){
        super(paused);

        if(!paused){
            _0PID.enableContinuousInput(-180, 180);
            SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());
            SmartDashboard.putData("Competition/Outtake Right Increase", Commands.runOnce(() -> _outtakeExtraMove += 0.005).withName("Right"));
            SmartDashboard.putData("Competition/Outtake Left Increase", Commands.runOnce(() -> _outtakeExtraMove -= 0.005).withName("Left"));
            SmartDashboard.putNumber("Competition/Outtake Extra Threshold", 0);
        }
    }

    public void resetSubsystem(){
        if(_paused)
           return;

        if(!RobotState.isAutonomous()){
            SwerveController.getInstance().setState("Driver");
            SwerveController.getInstance().setControl(new ChassisSpeeds(), false, "Driver");
        }
        else{
            SwerveController.getInstance().setState("Auto");
            SwerveController.getInstance().setControl(new ChassisSpeeds(), false, "Auto");
        }
    }

    @Override
    protected void setFunctionMaps() {

    }

    private Timer _atGoalTimer = new Timer();
    public Command goToReef(){
        Supplier<ChassisSpeeds> drive = () -> {
            Transform2d transform = RobotState.getInstance().getTransform(_currentReefTarget);
            double a = 3;
            double b = 2;

            double xVel = Math.pow(a * Math.abs(transform.getX()), 1 / b) * Math.signum(transform.getX());
            double yVel = Math.pow(a * Math.abs(transform.getY()), 1 / b) * Math.signum(transform.getY());
            double aVel = Math.pow(a / 2 * Math.abs(transform.getRotation().getRadians()), 1 / b) * Math.signum(transform.getRotation().getRadians());

            Logger.recordOutput("xTra", RobotState.getInstance().getTransform(_currentReefTarget).getX());
            Logger.recordOutput("yTra", RobotState.getInstance().getTransform(_currentReefTarget).getY());
            Logger.recordOutput("aTra", Units.degreesToRadians(RobotState.getInstance().getTransform(_currentReefTarget).getRotation().getDegrees()));
            Logger.recordOutput("xVel", xVel);
            Logger.recordOutput("yVel", yVel);
            Logger.recordOutput("aVel", aVel);

            return new ChassisSpeeds(
//                    _xPID.calculate(RobotState.getInstance().getRobotPose().getX(), _currentReefTarget.getX()),
//                    _yPID.calculate(RobotState.getInstance().getRobotPose().getY(), _currentReefTarget.getY()),
//                    _0PID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), _currentReefTarget.getRotation().getDegrees())
                    xVel,
                    yVel,
                    aVel
            );
        };

        return Commands.either(
                Commands.none(),
                Commands.sequence(
                        Commands.run(() -> {
                            _currentReefTarget = FieldConstants.getClosestReefTarget(
                                    RobotState.getInstance().isReefRight(),
                                    RobotState.getInstance().getReefLevel() == 4,
                                    _outtakeExtraMove);
                            _currentReefTarget = new Pose2d(_currentReefTarget.getTranslation(), _currentReefTarget.getRotation().rotateBy(Rotation2d.k180deg));
                            Logger.recordOutput("Reef Target", _currentReefTarget);
                        }, this).until(this::atPidingZone),
                        Commands.runOnce(() -> {
                            System.out.println("Reseting PID to X" + RobotState.getInstance().getRobotPose().getX() + " Y" + RobotState.getInstance().getRobotPose().getY() + " a" + RobotState.getInstance().getRobotPose().getRotation().getDegrees() + " vx" + SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond + " vy" + SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond + " omega" + SwerveIO.getInstance().getChassisSpeeds(true).omegaRadiansPerSecond);
                            _xPID.reset(RobotState.getInstance().getRobotPose().getX(), SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond);
                            _yPID.reset(RobotState.getInstance().getRobotPose().getY(), SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond);
                            _0PID.reset(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), Units.radiansToDegrees(SwerveIO.getInstance().getChassisSpeeds(true).omegaRadiansPerSecond));
                            SwerveController.getInstance().setState("Reef PID");
                        }, this),
                        Commands.run(() -> SwerveController.getInstance().setControl(drive.get(), true, "Reef PID"), this).until(this::atGoal),
                        Commands.runOnce(() -> {
                            _currentReefTarget = _currentReefTarget.transformBy(new Transform2d(0.03, 0, Rotation2d.kZero));
                            Logger.recordOutput("Reef Target", _currentReefTarget);
                        }, this),
                        Commands.run(() -> SwerveController.getInstance().setControl(drive.get(), true, "Reef PID"), this).until(this::atGoal),
                        Commands.runOnce(() -> _atGoalTimer.restart()),
                        Commands.run(() -> SwerveController.getInstance().setControl(drive.get(), true, "Reef PID"), this)
                        .until(() -> {
                            if(!atGoal())
                                _atGoalTimer.restart();
                            return _atGoalTimer.get() > 0.25;
                        })
                ),
                () -> RobotState.getInstance().getReefLevel() == 1
        );
    }

    public boolean atPidingZone(){
        return RobotState.getInstance().getDistance(_currentReefTarget) < FieldConstants.kStartPIDThreshold;
    }

    private boolean atGoalX(){
        if(!_paused){
            double thresh = FieldConstants.kXOuttakeDistThreshold + SmartDashboard.getNumber("Competition/Outtake Extra Threshold", 0) / 100;
            Logger.recordOutput("x dist", Math.abs(_currentReefTarget.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getY()));
            return Math.abs(_currentReefTarget.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getY()) < thresh;
        }
        return true;
    }

    private boolean atGoalY(){
        if(!_paused) {
            double thresh = FieldConstants.kYOuttakeDistThreshold;
            Logger.recordOutput("y dist", Math.abs(_currentReefTarget.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getX()));
            return Math.abs(_currentReefTarget.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getX()) < thresh;
        }
        return true;
    }

    private boolean atGoal0(){
        if(!_paused) {
            double angle = Math.abs(RobotState.getInstance().getTransform(_currentReefTarget).getRotation().getDegrees() % 360);
            Logger.recordOutput("0 dist", angle);
            return angle < FieldConstants.kOuttakeAngleThreshold;
        }
        return true;
    }

    private boolean atGoal(){
        if(!_paused)
            return atGoalX() && atGoalY() && atGoal0();
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_paused)
            return;

        SmartDashboard.putNumber("Competition/Dist To Reef", RobotState.getInstance()
                .getDistance(_currentReefTarget) * 100);
        SmartDashboard.putNumber("Competition/Outtake Extra Move", _outtakeExtraMove * 100);

        SwerveController.getInstance().periodic();
    }
}
