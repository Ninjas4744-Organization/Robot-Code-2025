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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends StateMachineSubsystem<RobotStates> {
    private static SwerveSubsystem _instance;

    public static SwerveSubsystem getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new SwerveSubsystem(paused);
    }

    private Pose2d _currentReefTarget = new Pose2d();
    private ProfiledPIDController _xPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3, 3.5));
    private ProfiledPIDController _yPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(3, 3.5));
    private ProfiledPIDController _0PID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(460, 1000));
    private double _outtakeExtraMove = 0;
    private Timer _forwardDriveTimer = new Timer();
    private boolean _preAtGoalX = false;
    private boolean isRight = false;
    private int stage = 1;
    private boolean startedPiding = false;

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

    public void setPidsToTeleop(){
        _xPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(2, 1.75));
        _yPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(2, 1.75));
        _0PID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(460, 1000));
        _0PID.enableContinuousInput(-180, 180);
    }

    public void setPidsToAuto(){
        _xPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(2, 1.75));
        _yPID = new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(2, 1.75));
        _0PID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(460, 1000));
        _0PID.enableContinuousInput(-180, 180);
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
          () -> {
              stage = 1;
              startedPiding = false;

              if(!RobotState.isAutonomous())
                SwerveController.getInstance().setState("Driver");
          },
          RobotStates.CLOSE,
          RobotStates.RESET);

        addFunctionToPeriodicMap(() -> {
            if(RobotState.getInstance().getReefLevel() == 1)
                return;

            _currentReefTarget = FieldConstants.getClosestReefTarget(
                    isRight,
                    RobotState.getInstance().getReefLevel() == 4,
                    _outtakeExtraMove);
            _currentReefTarget = new Pose2d(_currentReefTarget.getTranslation(), _currentReefTarget.getRotation().rotateBy(Rotation2d.k180deg));
            _currentReefTarget = _currentReefTarget.transformBy(new Transform2d(stage == 2 ? 0.03 : 0, 0, Rotation2d.kZero));
            Logger.recordOutput("Reef Target", _currentReefTarget);

            if(atPidingZone() && !startedPiding){
                System.out.println("Reseting PID to X" + RobotState.getInstance().getRobotPose().getX() + " Y" + RobotState.getInstance().getRobotPose().getY() + " a" + RobotState.getInstance().getRobotPose().getRotation().getDegrees() + " vx" + SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond + " vy" + SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond + " omega" + SwerveIO.getInstance().getChassisSpeeds(true).omegaRadiansPerSecond);
//                if(SmartDashboard.getBoolean("Competition/Enable Profile Init Velocity", false)){
                _xPID.reset(RobotState.getInstance().getRobotPose().getX(), SwerveIO.getInstance().getChassisSpeeds(true).vxMetersPerSecond);
                _yPID.reset(RobotState.getInstance().getRobotPose().getY(), SwerveIO.getInstance().getChassisSpeeds(true).vyMetersPerSecond);
                _0PID.reset(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), Units.radiansToDegrees(SwerveIO.getInstance().getChassisSpeeds(true).omegaRadiansPerSecond));
//                }else{
//                    _xPID.reset(RobotState.getInstance().getRobotPose().getX());
//                    _yPID.reset(RobotState.getInstance().getRobotPose().getY());
//                    _0PID.reset(RobotState.getInstance().getRobotPose().getRotation().getDegrees());
//                }
                isRight = RobotState.getInstance().getRobotState() == RobotStates.GO_RIGHT_REEF;
                startedPiding = true;
            }

            if(atGoal() && stage == 1)
                stage = 2;

            if(!atPidingZone())
                return;

            SwerveController.getInstance().setState("Reef PID");

            ChassisSpeeds pid = new ChassisSpeeds(
                    _xPID.calculate(RobotState.getInstance().getRobotPose().getX(), _currentReefTarget.getX()),
                    _yPID.calculate(RobotState.getInstance().getRobotPose().getY(), _currentReefTarget.getY()),
                    _0PID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), _currentReefTarget.getRotation().getDegrees())
            );
            SwerveController.getInstance().setControl(pid, true, "Reef PID");

//            double anglePid = _0PID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), _currentReefTarget.getRotation().getDegrees());
//            double forwardDrive = atGoalX() ? SwerveController.getInstance().pidTo(_currentReefTarget.getTranslation()).getNorm() : 0;//atGoalX() ? 1 : 0;
//            Logger.recordOutput("atGoalX", atGoalX());
//            Logger.recordOutput("forward timer", _forwardDriveTimer.get());
//
//            if(!_preAtGoalX && atGoalX())
//                _forwardDriveTimer.restart();
//            _preAtGoalX = atGoalX();

//            SwerveController.getInstance().setControl(
//                SwerveController.getInstance().lockAxis(_currentReefTarget.getRotation(), _currentReefTarget, new ChassisSpeeds(forwardDrive, 0, anglePid), false, false),
//                true,
//                "Reef PID"
//            );

//            Translation2d pidTrans = SwerveController.getInstance().pidTo(_currentReefTarget.getTranslation());
        }, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF, RobotStates.AT_SIDE_REEF, RobotStates.OUTTAKE_READY);

        addFunctionToOnChangeMap(() -> {
            SwerveController.getInstance().setState("Stop");
            SwerveController.getInstance().setControl(new ChassisSpeeds(), false, "Stop");
            stage = 1;
            startedPiding = false;
        },
        RobotStates.OUTTAKE);
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

    public boolean atReef(){
        if(!_paused)
            return atGoal() && stage == 2;
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_paused)
            return;

        atGoal();
        Logger.recordOutput("atGoalX", atGoalX());
        Logger.recordOutput("atGoalY", atGoalY());
        Logger.recordOutput("atGoal0", atGoal0());

        SmartDashboard.putNumber("Competition/Dist To Reef", RobotState.getInstance()
                .getDistance(_currentReefTarget) * 100);
        SmartDashboard.putNumber("Competition/Outtake Extra Move", _outtakeExtraMove * 100);

        SwerveController.getInstance().periodic();
    }
}
