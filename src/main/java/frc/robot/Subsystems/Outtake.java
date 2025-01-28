package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.StateMachine.RobotStates;

public class Outtake extends StateMachineMotoredSubsystem<RobotStates> {
    private static Outtake _instance;

    public static Outtake getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Outtake(paused);
    }

    public Outtake(boolean paused) {
        super(paused);
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
        if(!_paused)
            controller().stop();
    }

    @Override
    public boolean isResetted() {
        if(!_paused)
            return controller().getOutput() == 0;
        return true;
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> controller().setPercent(OuttakeConstants.kOuttakeState), RobotStates.OUTTAKE);
        addFunctionToOnChangeMap(() -> controller().setPercent(OuttakeConstants.kIntakeState), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> controller().setPercent(OuttakeConstants.kCloseState), RobotStates.CLOSE, RobotStates.RESET, RobotStates.OUTTAKE_READY);
    }
}
