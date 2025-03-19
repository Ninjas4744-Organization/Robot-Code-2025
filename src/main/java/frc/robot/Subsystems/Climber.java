package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstants;
import frc.robot.StateMachine.RobotStates;

import java.util.function.DoubleSupplier;

public class Climber extends StateMachineMotoredSubsystem<RobotStates> {
    private static Climber _instance;

    public static Climber getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Climber(paused);
    }

    public Climber(boolean paused) {
        super(paused);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(ClimberConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(ClimberConstants.kSimulatedControllerConstants);
    }

    @Override
    protected void resetSubsystemO() {
        controller().resetEncoder();
        controller().stop();
    }

    @Override
    protected boolean isResettedO() {
        return true;
    }

    @Override
    protected void setFunctionMaps() {

    }

    public Command stage1(){
        return Commands.runOnce(() -> controller().setPosition(ClimberConstants.kStage1), this);
    }

    public Command stage2(){
        return runMotor(1).until(controller()::getLimit);
    }
}
