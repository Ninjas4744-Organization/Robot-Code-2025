package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.HopperSushiConstans;
import frc.robot.StateMachine.RobotStates;

public class HopperSushi extends StateMachineMotoredSubsystem<RobotStates> {
    private static HopperSushi _instance;

    public static HopperSushi getInstance(){
        return _instance;
    }
    public HopperSushi(boolean paused) {
        super(paused);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(HopperSushiConstans.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(HopperSushiConstans.kSimulatedControllerConstants);

    }

    @Override
    protected void resetSubsystemO() {
        controller().stop();
    }

    @Override
    protected boolean isResettedO() {
        return controller().getOutput() == 0;
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> controller().setPosition(HopperSushiConstans.kIntakeState), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> controller().setPosition(HopperSushiConstans.kCloseState), RobotStates.RESET, RobotStates.CLOSE);
    }

}
