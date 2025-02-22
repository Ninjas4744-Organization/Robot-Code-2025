package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand.SwerveState;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private Pose2d _currentReefTag;
    private int isPIDInsteadOfDriveAssist = 0;

    private SwerveSubsystem(boolean paused){
        super(paused);

        if(!paused)
            SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
          () -> {
              SwerveIO.getInstance().setAccelerationLimit(SwerveConstants.kNormalAcc);
              isPIDInsteadOfDriveAssist = 0;

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
            Pose2d target = FieldConstants.getOffsetReefTagPose(_currentReefTag, RobotState.getInstance().getRobotState() == RobotStates.GO_RIGHT_REEF, RobotState.getInstance().getReefLevel() == 4);
            target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));
            SwerveController.getInstance().Demand.targetPose = target;

            Logger.recordOutput("Reef Target", target);
            Logger.recordOutput("isPIDInsteadOfDriveAssist", isPIDInsteadOfDriveAssist);
            if(isPIDInsteadOfDriveAssist == 2){
                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                SwerveController.getInstance().Demand.velocity = new ChassisSpeeds(pid.getX(), pid.getY(), SwerveController.getInstance().lookAt(target.getRotation().getDegrees(), 1));
                SwerveController.getInstance().Demand.fieldRelative = true;
                SwerveController.getInstance().setState(SwerveState.VELOCITY);
            }
            else if(isPIDInsteadOfDriveAssist == 1){
                SwerveController.getInstance().setState(SwerveState.DRIVE_ASSIST);
            }else{
                if(RobotState.getInstance().getDistanceTo(target).getNorm() <= SwerveConstants.kSwerveControllerConstants.driveAssistThreshold)
                    isPIDInsteadOfDriveAssist = RobotState.getInstance().getDistanceTo(target).getNorm() <= 0.2 ? 2 : 1;
            }

//            SwerveController.getInstance().Demand.point = target;
//            SwerveController.getInstance().Demand.angle = target.getRotation();
//            SwerveController.getInstance().setState(SwerveState.LOCKED_AXIS);

//            SwerveController.getInstance().Demand.targetPose = target;
//            SwerveController.getInstance().Demand.fieldRelative = true;
//            SwerveController.getInstance().Demand.velocity = new ChassisSpeeds(
//                SwerveController.getInstance().pidTo(target.getTranslation()).getX(),
//                SwerveController.getInstance().pidTo(target.getTranslation()).getY(),
//                SwerveController.getInstance().lookAt(target.getRotation().getDegrees(), 1)
//            );
//            SwerveController.getInstance().setState(SwerveState.VELOCITY);

        }, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);

        addFunctionToOnChangeMap(() -> {
            isPIDInsteadOfDriveAssist = 0;
            SwerveIO.getInstance().setAccelerationLimit(SwerveConstants.kNonFlippingAcc);
            SwerveController.getInstance().Demand.velocity = new ChassisSpeeds();
            SwerveController.getInstance().setState(SwerveState.VELOCITY);
        },
        RobotStates.AT_SIDE_REEF);
    }

    public boolean atReefSide(){
        if(!_paused)
            return RobotState.getInstance().getRobotPose().getTranslation()
                    .getDistance(SwerveController.getInstance().Demand.targetPose.getTranslation()) < 0.04;
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(!_paused)
            SwerveController.getInstance().periodic();
    }
}
