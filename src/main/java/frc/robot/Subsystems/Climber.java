package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClimberConstans;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.StateMachine.RobotStates;

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
        _simulatedController = new NinjasSimulatedController(ClimberConstans.kSimulatedControllerConstants);

    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(ClimberConstans.kSimulatedControllerConstants);
    }

    @Override
    protected void resetSubsystemO() {runMotor(ClimberConstans.kResetSpeed).until(_controller::getLimit).schedule();}

    @Override
    protected boolean isResettedO() {return !_controller.getLimit();}

    @Override
    protected void setFunctionMaps() {

    }
    public Command setPosition(double position){
        return Commands.runOnce(() -> controller().setPosition(position), this);
    }
}
