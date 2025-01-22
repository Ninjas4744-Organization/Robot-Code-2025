package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.HornAngleConstants;
import frc.robot.StateMachine.RobotStates;

public class HornAngle extends StateMachineMotoredSubsystem<RobotStates> {
    private static HornAngle _instance;
    private static boolean _dontCreate = false;


    public static HornAngle getInstance(){
        if(_instance == null)
            _instance = new HornAngle();
        return _instance;
    }

    @Override
    protected void setController() {
        if(!_dontCreate)
            _controller = new NinjasSparkMaxController(HornAngleConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        if(!_dontCreate)
            _simulatedController = new NinjasSimulatedController(HornAngleConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        if(!_dontCreate)
            controller().setPosition(HornAngleConstants.kOpenState);
    }

    @Override
    public boolean isResetted() {
        if(!_dontCreate)
            return controller().atGoal();
        return true;
    }

    public static void dontCreateSubsystem(){
        _dontCreate = true;
    }

    @Override
    protected void setFunctionMaps() {
        if(_dontCreate)
            return;
        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
    }
}
