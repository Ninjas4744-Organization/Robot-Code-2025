package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand.SwerveState;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class SwerveSubsystem extends StateMachineSubsystem<RobotStates> {
    private static SwerveSubsystem _instance;

    public static SwerveSubsystem getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new SwerveSubsystem(paused);
    }

    private Pose2d _currentReefTag;
    private final StructPublisher<Pose2d> _targetPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Reef Target", Pose2d.struct)
            .publish();

    private SwerveSubsystem(boolean paused){
        super(paused);

        if(!paused)
            SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
          () -> {
              if(!RobotState.isAutonomous())
                SwerveController.getInstance().setState(SwerveState.DEFAULT);
          },
          RobotStates.CORAL_READY,
          RobotStates.CORAL_SEARCH,
          RobotStates.OUTTAKE_READY);

        addFunctionToOnChangeMap(() -> {
            _currentReefTag = FieldConstants.getClosestReefTag();

            SwerveController.getInstance().Demand.targetPose = new Pose2d(_currentReefTag.getTranslation(), _currentReefTag.getRotation().rotateBy(Rotation2d.k180deg));
            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);

            _targetPosePublisher.set(SwerveController.getInstance().Demand.targetPose);
        }, RobotStates.L1, RobotStates.L2, RobotStates.L3, RobotStates.L4);

        addFunctionToPeriodicMap(() -> {
            Pose2d target = FieldConstants.getOffsetReefTagPose(_currentReefTag, RobotState.getInstance().getRobotState() == RobotStates.GO_RIGHT_REEF);
            target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));

//            SwerveController.getInstance().Demand.targetPose = target;
//            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);

//            SwerveController.getInstance().Demand.point = target;
//            SwerveController.getInstance().Demand.angle = target.getRotation();
//            SwerveController.getInstance().setState(SwerveState.LOCKED_AXIS);

            SwerveController.getInstance().Demand.targetPose = target;
            SwerveController.getInstance().Demand.velocity = new ChassisSpeeds(
                SwerveController.getInstance().pidTo(target.getTranslation()).getX(),
                SwerveController.getInstance().pidTo(target.getTranslation()).getY(),
                SwerveController.getInstance().lookAt(target.getRotation().getDegrees(), 1)
            );
            SwerveController.getInstance().setState(SwerveState.VELOCITY);

            _targetPosePublisher.set(SwerveController.getInstance().Demand.targetPose);
        }, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);
    }

    public boolean atReefSide(){
        if(!_paused)
            return RobotState.getInstance().getRobotPose().getTranslation()
                    .getDistance(SwerveController.getInstance().Demand.targetPose.getTranslation()) < 0.02;
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(!_paused)
            SwerveController.getInstance().periodic();
    }
}
