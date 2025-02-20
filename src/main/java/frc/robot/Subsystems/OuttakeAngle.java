package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.OuttakeAngleConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class OuttakeAngle extends StateMachineMotoredSubsystem<RobotStates> {
    private static OuttakeAngle _instance;

    public static OuttakeAngle getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new OuttakeAngle(paused);
    }

    private DigitalInput _limit;

    public OuttakeAngle(boolean paused) {
        super(paused);
        if(!_paused){
            _limit = new DigitalInput(OuttakeAngleConstants.kLimitSwitchID);
            Shuffleboard.getTab("Competition").addBoolean("Outtake Limit", _limit::get);
        }
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(OuttakeAngleConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(OuttakeAngleConstants.kSimulatedControllerConstants);

    }

    @Override
    protected void resetSubsystemO() {
        runMotor(-0.05).until(_limit::get).schedule();
    }

    @Override
    protected boolean isResettedO() {
        return _limit.get();
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() ->
            controller().setPercent(RobotState.getInstance().getReefLevel() != 1
            ? OuttakeAngleConstants.kCoralState
            : OuttakeAngleConstants.kL1State), RobotStates.AT_SIDE_REEF);

        addFunctionToOnChangeMap(() -> controller().setPosition(OuttakeAngleConstants.kCoralState), RobotStates.CLOSE, RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> controller().setPosition(OuttakeAngleConstants.kAlgaeState), RobotStates.REMOVE_ALGAE);
        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
    }
}
