package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.HornConstants;
import frc.robot.StateMachine.RobotStates;

public class Horn extends StateMachineMotoredSubsystem<RobotStates> {
    private static Horn _instance;

    public static Horn getInstance(){
        if(_instance == null)
            _instance = new Horn();
        return _instance;
    }

    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(HornConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(HornConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        controller().setPosition(HornConstants.kOpenState);
    }

    @Override
    public boolean isResetted() {
        return controller().atGoal();
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
    }
}
