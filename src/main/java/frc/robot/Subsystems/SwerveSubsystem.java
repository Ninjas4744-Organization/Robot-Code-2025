package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private final PIDController _xPID = new PIDController(9, 1.5, 0.1);
    private final PIDController _yPID = new PIDController(9, 1.5, 0.1);
    private final PIDController _0PID = new PIDController(0.1, 0, 0);
    private double _outtakeExtraMove = 0;
    private Timer _forwardDriveTimer = new Timer();
    private boolean _preAtGoalX = false;

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

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
          () -> {
              if(!RobotState.isAutonomous())
                SwerveController.getInstance().setState("Driver");
          },
          RobotStates.CLOSE,
          RobotStates.RESET);

        addFunctionToPeriodicMap(() -> {
            _currentReefTarget = FieldConstants.getClosestReefTarget(
                    RobotState.getInstance().getRobotState() == RobotStates.GO_RIGHT_REEF,
                    RobotState.getInstance().getReefLevel() == 4,
                    _outtakeExtraMove);
            _currentReefTarget = new Pose2d(_currentReefTarget.getTranslation(), _currentReefTarget.getRotation().rotateBy(Rotation2d.k180deg));
            Logger.recordOutput("Reef Target", _currentReefTarget);

            if(RobotState.getInstance().getDistance(_currentReefTarget) < 0.25)
                SwerveController.getInstance().setState("Reef PID");

//            SwerveController.getInstance().setControl(
//                new ChassisSpeeds(
//                    _xPID.calculate(RobotState.getInstance().getRobotPose().getX(), _currentReefTarget.getX()),
//                    _yPID.calculate(RobotState.getInstance().getRobotPose().getY(), _currentReefTarget.getY()),
//                    _0PID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), _currentReefTarget.getRotation().getDegrees())
//                ),
//                true,
//                "Reef PID"
//            );

            double anglePid = _0PID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), _currentReefTarget.getRotation().getDegrees());
            double forwardDrive = atGoalX() ? 1 : 0;
            Logger.recordOutput("atGoalX", atGoalX());
            Logger.recordOutput("forward timer", _forwardDriveTimer.get());

            if(!_preAtGoalX && atGoalX())
                _forwardDriveTimer.restart();
            _preAtGoalX = atGoalX();

            SwerveController.getInstance().setControl(
                SwerveController.getInstance().lockAxis(_currentReefTarget.getRotation(), _currentReefTarget, new ChassisSpeeds(forwardDrive, 0, anglePid), false, false),
                true,
                "Reef PID"
            );
        }, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);

        addFunctionToOnChangeMap(() -> {
            SwerveController.getInstance().setState("Stop");
            SwerveController.getInstance().setControl(new ChassisSpeeds(), false, "Stop");
        },
        RobotStates.AT_SIDE_REEF);
    }

    private boolean atGoalX(){
        if(!_paused){
            double thresh = (RobotState.getInstance().getRobotState() == RobotStates.GO_LEFT_REEF ? FieldConstants.kLeftOuttakeDistThreshold : FieldConstants.kRightOuttakeDistThreshold) + SmartDashboard.getNumber("Outtake Extra Threshold", 0) / 100;
            Logger.recordOutput("x dist", Math.abs(_currentReefTarget.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getY()));
            return Math.abs(_currentReefTarget.getTranslation().minus(RobotState.getInstance().getRobotPose().getTranslation()).rotateBy(FieldConstants.getClosestReefTag().getRotation().unaryMinus()).getY()) < thresh;
        }
        return true;
    }

    private boolean atGoalY(){
        if(!_paused)
            return _forwardDriveTimer.get() > 0.25;
        return true;
    }

    private boolean atGoal0(){
        if(!_paused) {
            double angle = RobotState.getInstance().getTransform(_currentReefTarget).getRotation().getDegrees() % 360;
            angle = (angle < 0) ? angle + 360 : angle;
            return angle < FieldConstants.kOuttakeAngleThreshold;
        }
        return true;
    }

    public boolean atGoal(){
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
