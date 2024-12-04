package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateMachine.RobotStates;

public class SwerveSubsystem extends StateMachineSubsystem<RobotStates> {
    private static SwerveSubsystem _instance;

    public static SwerveSubsystem getInstance(){
        if(_instance == null)
            _instance = new SwerveSubsystem();
        return _instance;
    }

    private SwerveSubsystem(){
//        SwerveIO.setConstants(SwerveConstants.kSwerveConstants);
//        SwerveController.setConstants(, SwerveIO.getInstance());
    }

    @Override
    protected void setFunctionMaps() {

    }

    @Override
    public void periodic() {
        super.periodic();
//        SwerveController.getInstance().periodic();
    }
}
