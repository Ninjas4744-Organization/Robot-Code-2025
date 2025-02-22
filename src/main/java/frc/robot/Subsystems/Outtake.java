package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class Outtake extends StateMachineMotoredSubsystem<RobotStates> {
    private static Outtake _instance;

    public static Outtake getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Outtake(paused);
    }

    private Outtake(boolean paused) {
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
    protected void resetSubsystemO() {
        controller().stop();
    }

    @Override
    protected boolean isResettedO() {
        return controller().getOutput() == 0;
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() ->
            controller().setVelocity(RobotState.getInstance().getReefLevel() != 1
            ? OuttakeConstants.kOuttakeState
            : OuttakeConstants.kL1OuttakeState)
        , RobotStates.OUTTAKE);

        addFunctionToPeriodicMap(() -> {
            if(controller().getCurrent() < 55)
                controller().setVelocity(OuttakeConstants.kIntakeState);
            else
                controller().setVelocity(OuttakeConstants.kIndexBackState);
        }, RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> controller().setVelocity(OuttakeConstants.kIndexBackState), RobotStates.INDEX_BACK);
        addFunctionToOnChangeMap(() -> controller().setVelocity(OuttakeConstants.kIndexState), RobotStates.INDEX);

        addFunctionToOnChangeMap(() -> controller().setVelocity(OuttakeConstants.kRemoveAlgae), RobotStates.REMOVE_ALGAE);
        addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CLOSE, RobotStates.RESET, RobotStates.CORAL_READY);
    }
}
