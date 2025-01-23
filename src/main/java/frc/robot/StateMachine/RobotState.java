package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.wpilibj.DigitalInput;

public class RobotState extends RobotStateWithSwerve<RobotStates> {
    private DigitalInput _beamBreaker = new DigitalInput(0);

    public RobotState(){
        _robotState = RobotStates.IDLE;
    }

    public boolean isCoralInRobot(){
        return !_beamBreaker.get();
    }

    public static RobotState getInstance() {
        return (RobotState)RobotStateWithSwerve.getInstance();
    }
}
