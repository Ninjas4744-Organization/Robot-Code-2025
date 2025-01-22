package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.StateMachine.RobotStates;

public class Outtake extends StateMachineMotoredSubsystem<RobotStates> {
    private static Outtake _instance;
    private static boolean _dontCreate = false;

    public static Outtake getInstance(){
        if(_instance == null)
            _instance = new Outtake();
        return _instance;
    }

    public static void dontCreateSubsystem(){
        _dontCreate = true;
    }

    @Override
    protected void setController(){
        if(!_dontCreate)
            _controller = new NinjasTalonFXController(OuttakeConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        if(!_dontCreate)
            _simulatedController = new NinjasSimulatedController(OuttakeConstants.kSimulatedControllerConstants);
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

        addFunctionToOnChangeMap(() -> controller().setPercent(OuttakeConstants.kOuttakeState), RobotStates.OUTTAKE);
        addFunctionToOnChangeMap(() -> controller().setPercent(OuttakeConstants.kIntakeState), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> controller().setPercent(OuttakeConstants.kCloseState), RobotStates.CLOSE, RobotStates.RESET);
    }
}