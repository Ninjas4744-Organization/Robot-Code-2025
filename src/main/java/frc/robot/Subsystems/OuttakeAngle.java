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
        runMotor(OuttakeAngleConstants.kResetSpeed).until(this::isResettedO).finallyDo(() -> controller().resetEncoder()).schedule();
    }

    @Override
    protected boolean isResettedO() {
        return controller().getLimit();
    }

    @Override
    protected void setFunctionMaps() {

    }

    public Command setPosition(DoubleSupplier position){
        return Commands.runOnce(() -> controller().setPosition(position.getAsDouble()));
    }

    public Command stop(){
        return Commands.runOnce(() -> controller().stop());
    }
}
