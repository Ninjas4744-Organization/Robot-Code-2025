package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.StateMachine.RobotStates;

public class Elevator extends StateMachineMotoredSubsystem<RobotStates> {
    private static Elevator _instance;
    private DigitalInput _limit;
    public static Elevator getInstance(){
        if(_instance == null)
            _instance = new Elevator();
        return _instance;
    }
    public Elevator (){
        _limit=new DigitalInput(ElevatorConstants.limitSwichid);

    }

    @Override
    protected void setController() {
        _controller= new NinjasTalonFXController(ElevatorConstants.kControllerConstants);

    }

    @Override
    protected void setSimulationController() {
        _controller= new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);

    }

    @Override
    public void resetSubsystem() {
        runMotor(-0.35).until(_limit::get).schedule();
    }

    @Override
    public boolean isResetted() {
        return _limit.get();
    }

    public double getPosition() {
        return controller().getPosition();
    }

    public double getGoal() {
        return controller().getGoal();
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
                ()->_controller.setPosition(ElevatorConstants.goollL1),RobotStates.L1
        );
        addFunctionToOnChangeMap(
                ()->_controller.setPosition(ElevatorConstants.goollL2),RobotStates.L2
        );
        addFunctionToOnChangeMap(
                ()->_controller.setPosition(ElevatorConstants.goollL3),RobotStates.L3
        );
        addFunctionToOnChangeMap(
                ()->_controller.setPosition(ElevatorConstants.goollL4),RobotStates.L4
        );
        addFunctionToOnChangeMap(
                ()->_controller.setPosition(ElevatorConstants.goollclose),RobotStates.CLOSE,RobotStates.RESET
        );



    }
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putBoolean("elevtor Limit", _limit.get());
        if (_limit.get()) {
            controller().resetEncoder();
            if (controller().getOutput() < 0) controller().stop();
        }
    }
}
