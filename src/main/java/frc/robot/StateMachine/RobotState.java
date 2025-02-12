package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Constants;

public class RobotState extends RobotStateWithSwerve<RobotStates> {
    private final DigitalInput _beamBreaker = new DigitalInput(Constants.kBeamBreakerID);
    private static int _reefLevel;

    public RobotState(){
        _robotState = RobotStates.IDLE;
    }

    public boolean isCoralInRobot(){
        if(!isSimulated())
            return !_beamBreaker.get();
        return false;
    }

    public void setReefLevel(int level) {_reefLevel = level;}

    public int getReefLevel() {return _reefLevel;}

    public static RobotState getInstance() {
        return (RobotState)RobotStateWithSwerve.getInstance();
    }
}
