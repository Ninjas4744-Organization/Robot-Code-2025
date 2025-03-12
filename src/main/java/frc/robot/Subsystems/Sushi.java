package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SushiConstants;
import frc.robot.StateMachine.RobotStates;

public class Sushi extends StateMachineMotoredSubsystem<RobotStates> {
    private static Sushi _instance;

    public static Sushi getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Sushi(paused);
    }

    public Sushi(boolean paused) {
        super(paused);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(SushiConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(SushiConstants.kSimulatedControllerConstants);
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
//        addFunctionToOnChangeMap(() -> controller().setPercent(SushiConstants.kIntake), RobotStates.INTAKE);
//        addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.RESET, RobotStates.CLOSE, RobotStates.CORAL_READY);
    }

    public Command setPercent(double percent){
        return Commands.runOnce(() -> controller().setPercent(percent), this);
    }
}
