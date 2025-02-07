package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.RobotStateIO;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class Elevator extends StateMachineMotoredSubsystem<RobotStates> {
    private static Elevator _instance;

    public static Elevator getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Elevator(paused);
    }

    private DigitalInput _limit;

    private Elevator (boolean paused){
        super(paused);

        if(!_paused)
            _limit = new DigitalInput(ElevatorConstants.kLimitSwitchID);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(ElevatorConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);
    }

    @Override
    protected void resetSubsystemO() {
        runMotor(-0.35).until(_limit::get).schedule();
    }

    @Override
    protected boolean isResettedO() {
        return _limit.get();
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> controller().setPosition(ElevatorConstants.kL1State), RobotStates.L1);
        addFunctionToOnChangeMap(() -> controller().setPosition(ElevatorConstants.kL2state), RobotStates.L2);
        addFunctionToOnChangeMap(() -> controller().setPosition(ElevatorConstants.kL3State), RobotStates.L3);
        addFunctionToOnChangeMap(() -> controller().setPosition(ElevatorConstants.kL4State), RobotStates.L4);

        addFunctionToOnChangeMap(() -> controller().setPosition(ElevatorConstants.kCloseState), RobotStates.CLOSE, RobotStates.RESET);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_paused)
            return;

        SmartDashboard.putBoolean("Elevator Limit", _limit.get());
        if (!RobotState.isSimulated() && _limit.get()) {
            controller().resetEncoder();
            if (controller().getOutput() < 0) controller().stop();
        }
    }
}
