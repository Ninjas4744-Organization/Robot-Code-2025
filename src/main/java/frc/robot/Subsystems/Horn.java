package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.HornConstants;
import frc.robot.StateMachine.RobotStates;

public class Horn extends StateMachineMotoredSubsystem<RobotStates> {
    private static Horn _instance;
    private static boolean _dontCreate = false;

    public static Horn getInstance(){
        if(_instance == null)
            _instance = new Horn();
        return _instance;
    }

    public static void dontCreateSubsystem(){
        _dontCreate = true;
    }

    @Override
    protected void setController() {
        if(!_dontCreate)
            _controller = new NinjasSparkMaxController(HornConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        if(!_dontCreate)
            _simulatedController = new NinjasSimulatedController(HornConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        if(!_dontCreate)
            controller().stop();
    }

    @Override
    public boolean isResetted() {
        if(!_dontCreate)
            return controller().atGoal();
        return true;
    }

    @Override
    protected void setFunctionMaps() {
        if(_dontCreate)
            return;

        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET, RobotStates.IDLE);
        addFunctionToOnChangeMap(() -> controller().setPercent(HornConstants.kSpeedPercent), RobotStates.REMOVE_ALGAE);
    }

    @Override
    public void periodic() {
        if(!_dontCreate)
            super.periodic();
    }
}
