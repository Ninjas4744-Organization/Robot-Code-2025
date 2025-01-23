package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.StateMachine.RobotStates;

public class Elevator extends StateMachineMotoredSubsystem<RobotStates> {
    private static Elevator _instance;
    private static boolean _dontCreate = false;

    public static Elevator getInstance(){
        if(_instance == null)
            _instance = new Elevator();
        return _instance;
    }

    public static void dontCreateSubsystem(){
        _dontCreate = true;
        getInstance();
    }

    private DigitalInput _limit;

    public Elevator (){
        super();
        if(!_dontCreate)
            _limit = new DigitalInput(ElevatorConstants.kLimitSwitchID);
    }

    @Override
    protected void setController() {
        if(!_dontCreate)
            _controller = new NinjasTalonFXController(ElevatorConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        if(!_dontCreate)
            _simulatedController = new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        if(!_dontCreate)
            runMotor(-0.35).until(_limit::get).schedule();
    }

    @Override
    public boolean isResetted() {
        if(!_dontCreate)
            return _limit.get();
        return true;
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
        if(_dontCreate)
            return;

        super.periodic();

        SmartDashboard.putBoolean("Elevator Limit", _limit.get());
        if (_limit.get()) {
            controller().resetEncoder();
            if (controller().getOutput() < 0) controller().stop();
        }
    }
}
