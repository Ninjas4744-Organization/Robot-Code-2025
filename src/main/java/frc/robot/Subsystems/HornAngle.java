package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.HornAngleConstants;
import frc.robot.StateMachine.RobotStates;

public class HornAngle extends StateMachineMotoredSubsystem<RobotStates> {
    private static HornAngle _instance;

    public static HornAngle getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new HornAngle(paused);
    }

    public HornAngle(boolean paused) {
        super(paused);
    }

    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(HornAngleConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(HornAngleConstants.kSimulatedControllerConstants);
    }

    @Override
    protected void resetSubsystemO() {
        if(!_paused)
            controller().setPosition(HornAngleConstants.kOpenState);
    }

    @Override
    protected boolean isResettedO() {
        if(!_paused)
            return controller().atGoal();
        return true;
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
    }
}
