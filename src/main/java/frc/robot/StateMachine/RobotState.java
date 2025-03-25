package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Constants;
import org.littletonrobotics.junction.Logger;

public class RobotState extends RobotStateWithSwerve<RobotStates> {
    private final DigitalInput _beamBreaker = new DigitalInput(Constants.kBeamBreakerID);
    private static int _reefLevel;
    private static boolean _removeAlgae;
    private static boolean _isReefRight;

    public RobotState(){
        _robotState = RobotStates.IDLE;
        setReefLevel(3);
        setReefRight(false);
        setAlgae(false);
    }

    public boolean isCoralInRobot(){
        if(!isSimulated())
            return !_beamBreaker.get();
        return false;
    }

    public void setReefLevel(int level) {
        _reefLevel = level;
        Logger.recordOutput("Other/Reef Level", level);
    }

    public void setAlgae(boolean remove) {
        _removeAlgae = remove;
        Logger.recordOutput("Other/Remove Algae", _removeAlgae);
    }

    public int getReefLevel() {
        return _reefLevel;
    }

    public void setReefRight(boolean reefRight){
        _isReefRight = reefRight;
        Logger.recordOutput("Other/Is Reef Right", reefRight);
    }

    public boolean isReefRight(){
        return _isReefRight;
    }

    public boolean getAlgae() {
        Logger.recordOutput("Other/Remove Algae", _removeAlgae);
        return _removeAlgae;
    }

    public static RobotState getInstance() {
        return (RobotState)RobotStateWithSwerve.getInstance();
    }
}
