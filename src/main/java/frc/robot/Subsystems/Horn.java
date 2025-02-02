package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.HornConstants;
import frc.robot.StateMachine.RobotStates;

public class Horn extends StateMachineMotoredSubsystem<RobotStates> {
    private static Horn _instance;

    public static Horn getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Horn(paused);
    }

    private Horn(boolean paused) {
        super(paused);
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
    protected void resetSubsystemO() {
        if(!_paused)
            controller().stop();
    }

    @Override
    protected boolean isResettedO() {
        if(!_paused)
            return controller().getOutput() == 0;
        return true;
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
        addFunctionToOnChangeMap(() -> controller().setPercent(HornConstants.kSpeedPercent), RobotStates.REMOVE_ALGAE);
    }
}
