package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand.SwerveState;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
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

    private SwerveSubsystem(){
        SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());
        SwerveController.getInstance().setState(SwerveState.DEFAULT);
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
          () -> {
              if(!RobotState.isAutonomous())
                SwerveController.getInstance().setState(SwerveState.DEFAULT);
          },
          RobotStates.IDLE,
          RobotStates.CLOSE,
          RobotStates.RESET,
          RobotStates.NOTE_IN_INDEXER,
          RobotStates.NOTE_SEARCH);

        addFunctionToPeriodicMap(
          () -> {
              SwerveController.getInstance()._demand.targetPose = FieldConstants.getOffsetTagPose(
                FieldConstants.getTagPose(FieldConstants.getAmpTag().ID)
                  .toPose2d(),
                0.75);

              SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);

//              double dist =
//                RobotState.getInstance().getRobotPose().getTranslation().getDistance(_demand.targetPose.getTranslation());
//              if (dist < SwerveConstants.kPathFollowerDistThreshold) SwerveController.getInstance().setState(SwerveState.FOLLOW_PATH);
          },
          RobotStates.DRIVE_TO_AMP);

        addFunctionToOnChangeMap(
          () -> {
              SwerveController.getInstance().setState(SwerveState.LOOK_AT_TARGET);
              SwerveController.getInstance()._demand.targetPose = FieldConstants.getTagPose(FieldConstants.getSpeakerTag().ID).toPose2d();
          },
          RobotStates.SHOOT_SPEAKER_PREPARE,
          RobotStates.DELIVERY);

//        addFunctionToPeriodicMap(
//          () -> {
//              SwerveController.getInstance()._demand.targetPose = FieldConstants.getOffsetTagPose(
//                FieldConstants.getTagPose(FieldConstants.getSourceTag().ID)
//                  .toPose2d(),
//                1.25);
//
//              double dist =
//                RobotState.getInstance().getRobotPose().getTranslation().getDistance(_demand.targetPose.getTranslation());
//              if (dist < SwerveConstants.kPathFollowerDistThreshold) SwerveController.getInstance().setState(SwerveState.FOLLOW_PATH);
//          },
//          RobotStates.DRIVE_TO_SOURCE);

        //		addFunctionToOnChangeMap(
        //				() -> {
        //					SwerveController.getInstance().setState(SwerveState.LOCKED_AXIS);
        //					updateDemand(Rotation2d.fromDegrees(90), 1.84, false);
        //				},
        //				RobotStates.SHOOT_AMP_PREPARE);

//        addFunctionToOnChangeMap(() -> SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST), RobotStates.NOTE_SEARCH);
    }

    @Override
    public void periodic() {
        super.periodic();
        SwerveController.getInstance().periodic();
        SwerveIO.getInstance().periodic();
    }
}
