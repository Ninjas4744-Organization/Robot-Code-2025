package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand.SwerveState;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
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

    private Pose2d _currentReefTag;
    private int _isPIDInsteadOfDriveAssist = 0;
    private PIDController _xEndPID = new PIDController(8, 1.5 ,0.1);
    private PIDController _yEndPID = new PIDController(8, 1.5 ,0.1);
    private PIDController _0EndPID = new PIDController(0.1, 0, 0);

    private SwerveSubsystem(boolean paused){
        super(paused);

        if(!paused){
            SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());
//            RobotContainer.logToShuffleboard("L Change", 0);
//            RobotContainer.logToShuffleboard("L Change Reset", false);
        }
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
          () -> {
              SwerveIO.getInstance().setAccelerationLimit(SwerveConstants.kNormalAcc);
              _isPIDInsteadOfDriveAssist = 0;

              if(!RobotState.isAutonomous())
                SwerveController.getInstance().setState(SwerveState.DEFAULT);
          },
          RobotStates.CLOSE,
          RobotStates.RESET);

//        addFunctionToOnChangeMap(() -> {
//            _currentReefTag = FieldConstants.getClosestReefTag();
//
//            SwerveController.getInstance().Demand.targetPose = new Pose2d(_currentReefTag.getTranslation(), _currentReefTag.getRotation().rotateBy(Rotation2d.k180deg));
//            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);
//
//            Logger.recordOutput("Reef Target", SwerveController.getInstance().Demand.targetPose);
//        }, RobotStates.);

        addFunctionToPeriodicMap(() -> {
            _currentReefTag = FieldConstants.getClosestReefTag();
            Pose2d target = FieldConstants.getOffsetReefTagPose(_currentReefTag,
                    RobotState.getInstance().getRobotState() == RobotStates.GO_RIGHT_REEF,
                    RobotState.getInstance().getReefLevel() == 4,
                    0/*RobotContainer.getFromShuffleboard("L Change").getDouble()*/);

            target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));
            SwerveController.getInstance().Demand.targetPose = target;

            Logger.recordOutput("Reef Target", target);
            Logger.recordOutput("isPIDInsteadOfDriveAssist", _isPIDInsteadOfDriveAssist);
            if(_isPIDInsteadOfDriveAssist == 2){
                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                SwerveController.getInstance().Demand.velocity =
                        new ChassisSpeeds(_xEndPID.calculate(RobotState.getInstance().getRobotPose().getX(), target.getX()),
                                _yEndPID.calculate(RobotState.getInstance().getRobotPose().getY(), target.getY()),
                                _0EndPID.calculate(RobotState.getInstance().getRobotPose().getRotation().getDegrees(), target.getRotation().getDegrees()));
                SwerveController.getInstance().Demand.fieldRelative = true;
                SwerveController.getInstance().setState(SwerveState.VELOCITY);
            }
            else if(_isPIDInsteadOfDriveAssist == 1){
                SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);
                if(SwerveController.getInstance().isDriveAssistFinished())
                    _isPIDInsteadOfDriveAssist = 2;
            }else{
                if(RobotState.getInstance().getDistanceTo(target).getNorm() <= SwerveConstants.kSwerveControllerConstants.driveAssistThreshold)
                    _isPIDInsteadOfDriveAssist = RobotState.getInstance().getDistanceTo(target).getNorm() <= 0.2 ? 2 : 1;
            }
        }, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);

        addFunctionToPeriodicMap(() -> {
            _currentReefTag = FieldConstants.getClosestReefTag();
            Pose2d target = _currentReefTag;
            target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));

            SwerveController.getInstance().Demand.targetPose = target;
            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);

            Logger.recordOutput("Reef Target", target);
        }, RobotStates.GO_ALGAE);

        addFunctionToOnChangeMap(() -> {
            _currentReefTag = FieldConstants.getClosestReefTag().transformBy(new Transform2d(0, -0.2, Rotation2d.kZero));
            Pose2d target = _currentReefTag;
            target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));

            SwerveController.getInstance().Demand.targetPose = target;
            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);

            Logger.recordOutput("Reef Target", target);
        }, RobotStates.GO_ALGAE_BACK);

        addFunctionToOnChangeMap(() -> {
            _isPIDInsteadOfDriveAssist = 0;
            SwerveIO.getInstance().setAccelerationLimit(SwerveConstants.kNonFlippingAcc);
            SwerveController.getInstance().Demand.velocity = new ChassisSpeeds();
            SwerveController.getInstance().setState(SwerveState.VELOCITY);
        },
        RobotStates.AT_SIDE_REEF);
    }

    public boolean atGoal(){
        if(!_paused)
            return RobotState.getInstance().getRobotPose().getTranslation()
                    .getDistance(SwerveController.getInstance().Demand.targetPose.getTranslation()) < FieldConstants.kOuttakeDistThreshold
                    && Math.abs(SwerveController.getInstance().Demand.targetPose.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) < FieldConstants.kOuttakeAngleThreshold;
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_paused)
            return;

        RobotContainer.logToShuffleboard("Dist To Reef", RobotState.getInstance().getRobotPose().getTranslation()
                .getDistance(SwerveController.getInstance().Demand.targetPose.getTranslation()));

//        if(RobotContainer.getFromShuffleboard("L Change Reset").getBoolean()){
//            RobotContainer.logToShuffleboard("L Change", 0);
//            RobotContainer.logToShuffleboard("L Change Reset", false);
//        }

        SwerveController.getInstance().periodic();
    }
}
