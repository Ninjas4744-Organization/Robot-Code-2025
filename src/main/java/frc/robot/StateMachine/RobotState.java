package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Constants;
import org.littletonrobotics.junction.Logger;

public class RobotState extends RobotStateWithSwerve<RobotStates> {
    private final DigitalInput _beamBreaker = new DigitalInput(Constants.kBeamBreakerID);
    private static int _reefLevel;
    private static int _algaeLevel;

    public RobotState(){
        _robotState = RobotStates.IDLE;
        setReefLevel(3);
        setAlgaeLevel(2);
    }

    public boolean isCoralInRobot(){
        if(!isSimulated())
            return !_beamBreaker.get();
        return false;
    }

    public void setAlgaeLevel(int level) {
        _algaeLevel = level;
        Logger.recordOutput("Algae Level", level);
    }

    public int getAlgaeLevel() {
        return _algaeLevel;
    }

    public void setReefLevel(int level) {
        _reefLevel = level;
        Logger.recordOutput("Reef Level", level);
    }

    public int getReefLevel() {
        return _reefLevel;
    }

    public static RobotState getInstance() {
        return (RobotState)RobotStateWithSwerve.getInstance();
    }
}
