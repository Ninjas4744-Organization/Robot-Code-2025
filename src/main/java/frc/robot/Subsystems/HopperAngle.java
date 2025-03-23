package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HopperAngleConstants;
import frc.robot.StateMachine.RobotStates;

import java.util.function.DoubleSupplier;

public class HopperAngle extends StateMachineMotoredSubsystem<RobotStates> {
    private static HopperAngle _instance;

    public static HopperAngle getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new HopperAngle(paused);
    }

    public HopperAngle(boolean paused) {
        super(paused);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(HopperAngleConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(HopperAngleConstants.kSimulatedControllerConstants);
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

    public Command setPosition(DoubleSupplier position){
        return Commands.runOnce(() -> controller().setPosition(position.getAsDouble()));
    }
}
