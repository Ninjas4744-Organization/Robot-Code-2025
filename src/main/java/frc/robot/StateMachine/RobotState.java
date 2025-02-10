package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Constants;

public class RobotState extends RobotStateWithSwerve<RobotStates> {
    private final DigitalInput _beamBreaker = new DigitalInput(Constants.kBeamBreakerID);

    public RobotState(){
        _robotState = RobotStates.IDLE;
    }

    public boolean isCoralInRobot(){
        if(!isSimulated())
            return !_beamBreaker.get();
        return false;
    }

    public static RobotState getInstance() {
        return (RobotState)RobotStateWithSwerve.getInstance();
    }
}
