package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OuttakeRotationConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class OuttakeRotation extends StateMachineMotoredSubsystem<RobotStates> {
    private static OuttakeRotation _instance;
    private DigitalInput _limit;
    public static void createInstance(boolean paused){
        _instance = new OuttakeRotation(paused);
    }

    public OuttakeRotation(boolean paused) {
        super(paused);
        if(!_paused){
            _limit = new DigitalInput(ElevatorConstants.kLimitSwitchID);
            Shuffleboard.getTab("Competition").addBoolean("Elevator Limit", _limit::get);
        }
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(OuttakeRotationConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(OuttakeRotationConstants.kSimulatedControllerConstants);

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
        addFunctionToOnChangeMap(() ->
                        controller().setPosition(OuttakeRotationConstants.kAngleStates[1]),
                RobotStates.CLOSE);
        addFunctionToOnChangeMap(() ->
                        controller().setPosition(OuttakeRotationConstants.kAngleStates[0]),
                RobotStates.CORAL_SEARCH,RobotStates.OUTTAKE_READY,RobotStates.RESET);


    }
}
