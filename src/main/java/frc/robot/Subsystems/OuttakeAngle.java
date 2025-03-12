package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OuttakeAngleConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class OuttakeAngle extends StateMachineMotoredSubsystem<RobotStates> {
    private static OuttakeAngle _instance;

    public static OuttakeAngle getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new OuttakeAngle(paused);
    }

    public OuttakeAngle(boolean paused) {
        super(paused);
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
        runMotor(OuttakeAngleConstants.kResetSpeed).until(_controller::getLimit).schedule();
    }

    @Override
    protected boolean isResettedO() {
        return _controller.getLimit();
    }

    @Override
    protected void setFunctionMaps() {
//        addFunctionToOnChangeMap(() ->
//            controller().setPosition(RobotState.getInstance().getReefLevel() != 1
//            ? OuttakeAngleConstants.kCoralState
//            : OuttakeAngleConstants.kL1State), RobotStates.AT_REEF);
//
//        addFunctionToOnChangeMap(() -> controller().setPosition(OuttakeAngleConstants.kCoralState), RobotStates.INTAKE);
//
//        addFunctionToOnChangeMap(() -> Commands.run(() -> controller().setPosition(OuttakeAngleConstants.kCoralState)).until(() -> controller().atGoal()).andThen(runMotor(OuttakeAngleConstants.kResetSpeed)).until(_limit::get).schedule(), RobotStates.CLOSE);
//        addFunctionToOnChangeMap(() -> controller().setPosition(OuttakeAngleConstants.kAlgaeState), RobotStates.REMOVE_ALGAE);
//        addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
    }

    public Command setPosition(DoubleSupplier position){
        return Commands.runOnce(() -> controller().setPosition(position.getAsDouble()), this);
    }
}
