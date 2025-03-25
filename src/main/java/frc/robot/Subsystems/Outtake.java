package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

import java.util.function.DoubleSupplier;

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

    }

    public Command setVelocity(DoubleSupplier velocity){
        return Commands.runOnce(() -> controller().setVelocity(velocity.getAsDouble()));
    }

    public Command outtake(){
        return Commands.runOnce(() -> {
            switch (RobotState.getInstance().getReefLevel()){
                case 1:
                    controller().setVelocity(OuttakeConstants.kL1OuttakeState);
                    break;

                case 4:
                    controller().setVelocity(OuttakeConstants.kL4OuttakeState);
                    break;

                default:
                    controller().setVelocity(OuttakeConstants.kOuttakeState);
                    break;
            }
        });
    }

    public Command intake(){
        return Commands.run(() -> {
            if(controller().getCurrent() > 69)
                controller().setVelocity(OuttakeConstants.kOuttakeBackState);
            else
               controller().setVelocity(OuttakeConstants.kIntakeState);
        });
    }
}
