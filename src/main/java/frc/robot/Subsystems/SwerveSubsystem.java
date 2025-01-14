package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand.SwerveState;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class SwerveSubsystem extends StateMachineSubsystem<RobotStates> {
    private static SwerveSubsystem _instance;

    public static SwerveSubsystem getInstance(){
        if(_instance == null)
            _instance = new SwerveSubsystem();
        return _instance;
    }

    private AprilTag _currentReefTag;
    private final StructPublisher<Pose2d> _targetPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Reef Target", Pose2d.struct)
            .publish();

    private SwerveSubsystem(){
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
            _targetPosePublisher.set(_currentReefTag.pose.toPose2d());

            SwerveController.getInstance().Demand.targetPose = _currentReefTag.pose.toPose2d();
            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);
        }, RobotStates.L1, RobotStates.L2, RobotStates.L3, RobotStates.L4);

        addFunctionToOnChangeMap(() -> {
            Pose2d target = FieldConstants.getOffsetReefTagPose(_currentReefTag, RobotState.getInstance().getRobotState() == RobotStates.GO_RIGHT_REEF);
            _targetPosePublisher.set(target);

            SwerveController.getInstance().Demand.targetPose = target;
            SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);
        }, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);
    }

    @Override
    public void periodic() {
        super.periodic();
        SwerveController.getInstance().periodic();
    }
}
