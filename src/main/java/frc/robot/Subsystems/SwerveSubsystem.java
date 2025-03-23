package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        if(getCurrentCommand() != null)
            getCurrentCommand().cancel();
    }

    @Override
    protected void setFunctionMaps() {

    }

//    private Timer _atGoalTimer = new Timer();
    double t = 0;
    double time = 0.75;
    boolean pastP1Start = false;
//    PIDController _xp = new PIDController(5, 0, 0);
//    PIDController _yp = new PIDController(5, 0, 0);
    Pose2d p0;
    Pose2d p1;
    Pose2d p2;
    public Command goToReef(){
        Supplier<ChassisSpeeds> drive = () -> {
//            if(t == 0){
//                p0 = RobotState.getInstance().getRobotPose();
//                p2 = _currentReefTarget;
//                p1 = _currentReefTarget.transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kZero));
//            }
//
            Pose2d[] poses = new Pose2d[11];
            for (double t = 0; t <= 1; t += 0.1) {
                int i = (int)Math.round(t * 10);
                poses[i] = p0.interpolate(p1, t).interpolate(p1.interpolate(p2, t), t);
            }
            Logger.recordOutput("trajectory", poses);

            t += 0.02 / time;
            Pose2d target = p0.interpolate(p1, t).interpolate(p1.interpolate(p2, t), t);

            Transform2d transform1 = RobotState.getInstance().getTransform(_currentReefTarget);
            Transform2d transform2 = RobotState.getInstance().getTransform(target);
            double a = 1.75;//1.75;
            double b = 1.75;//1.75;

            if(pastP1Start)
                transform2 = transform1;

            double vel = Math.pow(a * Math.abs(transform1.getTranslation().getNorm()), 1 / b);
            double x = transform1.getTranslation().getNorm();
//            double vel = -1.34 * x * x * x * x + 6.55 * x * x * x + -9.99 * x * x + 6.78 * x;
            double xVel = vel * transform2.getTranslation().getAngle().getCos();
            double yVel = vel * transform2.getTranslation().getAngle().getSin();
//            double xVel = _xp.calculate(-transform.getX());
//            double yVel = _yp.calculate(-transform.getY());

            return new ChassisSpeeds(
                    xVel,
                    yVel,
                    _0PID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), _currentReefTarget.getRotation().getDegrees())
            );
        };

        return Commands.sequence(
            Commands.run(() -> {
                _currentReefTarget = FieldConstants.getClosestReefTarget(
                        RobotState.getInstance().isReefRight(),
                        _outtakeExtraMove);
                _currentReefTarget = new Pose2d(_currentReefTarget.getTranslation(), _currentReefTarget.getRotation().rotateBy(Rotation2d.k180deg));
                Logger.recordOutput("Reef Target", _currentReefTarget);
            }).until(this::atPidingZone),
            Commands.runOnce(() -> {
                System.out.println("Reseting PID to X" + RobotState.getInstance().getRobotPose().getX() + " Y" + RobotState.getInstance().getRobotPose().getY() + " a" + RobotState.getInstance().getRobotPose().getRotation().getDegrees() + " vx" + SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond + " vy" + SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond + " omega" + SwerveIO.getInstance().getChassisSpeeds(true).omegaRadiansPerSecond);
                _xPID.reset(RobotState.getInstance().getRobotPose().getX(), SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond);
                _yPID.reset(RobotState.getInstance().getRobotPose().getY(), SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond);
                _0PID.reset(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), Units.radiansToDegrees(SwerveIO.getInstance().getChassisSpeeds(true).omegaRadiansPerSecond));
                t = 0.05;
                p0 = RobotState.getInstance().getRobotPose();
                p2 = _currentReefTarget;
                Translation2d velDir = new Translation2d(
                        SwerveIO.getInstance().getChassisSpeeds(false).vxMetersPerSecond,
                        SwerveIO.getInstance().getChassisSpeeds(false).vyMetersPerSecond);
                if(velDir.getNorm() < 5){
                    p1 = _currentReefTarget.transformBy(new Transform2d(-0.75, 0.0, Rotation2d.kZero));
                    pastP1Start = afterP1();
                }else{
                    velDir = new Translation2d(velDir.getX() * 0.4, velDir.getY() * 0.4);
                    p1 = RobotState.getInstance().getRobotPose().transformBy(new Transform2d(velDir.getX(), velDir.getY(), Rotation2d.kZero));
                    pastP1Start = false;
                }
                SwerveController.getInstance().setState("Reef PID");
            }),
            Commands.run(() -> SwerveController.getInstance().setControl(drive.get(), true, "Reef PID"), this)
//                        Commands.runOnce(() -> {
//                            _currentReefTarget = _currentReefTarget.transformBy(new Transform2d(0.03, 0, Rotation2d.kZero));
//                            Logger.recordOutput("Reef Target", _currentReefTarget);
//                        }, this),
//                        Commands.run(() -> SwerveController.getInstance().setControl(drive.get(), true, "Reef PID"), this).until(this::atGoal)
//                        Commands.runOnce(() -> _atGoalTimer.restart()),
//                        Commands.run(() -> SwerveController.getInstance().setControl(drive.get(), true, "Reef PID"), this).until(this::atGoal)
//                        .until(() -> {
//                            if(!atGoal())
//                                _atGoalTimer.restart();
//                            return _atGoalTimer.get() > 0.25;
//                        })
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

    private boolean afterP1(){
        if(!_paused) {
            Logger.recordOutput("p1 dist", p1.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getX());
            return p1.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getX() > 0;
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

    public boolean atGoal(){
        if(!_paused)
            return atGoalX() && atGoalY() && atGoal0();
        return true;
    }

    public double distToTarget(){
        Logger.recordOutput("Dist", RobotState.getInstance().getDistance(FieldConstants.getClosestReefTarget(
                RobotState.getInstance().isReefRight(),
                _outtakeExtraMove)));
        return RobotState.getInstance().getDistance(FieldConstants.getClosestReefTarget(
                RobotState.getInstance().isReefRight(),
                _outtakeExtraMove));
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
