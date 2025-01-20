package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.StateMachine.RobotStates;

public class Outtake extends StateMachineMotoredSubsystem<RobotStates> {
    private static Outtake _instance;

    public static Outtake getInstance(){
        if(_instance == null)
            _instance = new Outtake();
        return _instance;
    }

    @Override
    protected void setController(){
        _controller = new NinjasTalonFXController(OuttakeConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(OuttakeConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        _controller.stop();
    }

    @Override
    public boolean isResetted() {
        return _controller.getOutput() == 0;
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> _controller.setPercent(OuttakeConstants.kOuttakeState), RobotStates.OUTTAKE);
        addFunctionToOnChangeMap(() -> _controller.setPercent(OuttakeConstants.kIntakeState), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> _controller.setPercent(OuttakeConstants.kCLoseState), RobotStates.CLOSE, RobotStates.RESET);
    }
}