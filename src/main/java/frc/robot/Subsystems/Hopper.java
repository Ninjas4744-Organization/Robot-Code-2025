package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.HopperConstants;
import frc.robot.StateMachine.RobotStates;

public class Hopper extends StateMachineMotoredSubsystem<RobotStates> {
    private static Hopper _instance;
    private static  boolean _dontCreate = false;

    public static Hopper getInstance(){
        if(_instance == null)
            _instance = new Hopper();
        return _instance;
    }

    public static void dontCreateSubsystem() {
        _dontCreate = true;
    }

    @Override
    protected void setController() {
        if(!_dontCreate)
            _controller = new NinjasTalonFXController(HopperConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        if(!_dontCreate)
            _simulatedController = new NinjasSimulatedController(HopperConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        if(!_dontCreate)
            controller().stop();
    }

    @Override
    public boolean isResetted() {
        if(!_dontCreate)
            return controller().getOutput() == 0;
        return true;
    }

    @Override
    protected void setFunctionMaps() {
        if(_dontCreate)
            return;

        addFunctionToOnChangeMap(() -> controller().setPercent(HopperConstants.kIntakeState), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> controller().setPercent(HopperConstants.kCloseState), RobotStates.CLOSE, RobotStates.RESET);
    }

    @Override
    public void periodic() {
        if(!_dontCreate)
            super.periodic();
    }
}
