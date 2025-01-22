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
    private static boolean _dontCreate = false;

    public static Elevator getInstance(){
        if(_instance == null)
            _instance = new Elevator();
        return _instance;
    }

    private DigitalInput _limit;

    public Elevator (){
        super();
        if(!_dontCreate)
            _limit=new DigitalInput(ElevatorConstants.limitSwichid);
    }

    @Override
    protected void setController() {
        if(!_dontCreate)
            _controller= new NinjasTalonFXController(ElevatorConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        if(!_dontCreate)
            _controller= new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);
    }

    @Override
    public void resetSubsystem() {
        if(!_dontCreate)
            runMotor(-0.35).until(_limit::get).schedule();
    }

    @Override
    public boolean isResetted() {
        if(!_dontCreate)
            return _limit.get();
        return true;
    }

    public static void dontCreateSubsystem(){
        _dontCreate = true;
    }

    public double getPosition() {
        return controller().getPosition();
    }

    public double getGoal() {
        return controller().getGoal();
    }

    @Override
    protected void setFunctionMaps() {
        if(_dontCreate)
            return;

        addFunctionToOnChangeMap(
                ()->controller().setPosition(ElevatorConstants.goollL1),RobotStates.L1
        );
        addFunctionToOnChangeMap(
                ()->controller().setPosition(ElevatorConstants.goollL2),RobotStates.L2
        );
        addFunctionToOnChangeMap(
                ()->controller().setPosition(ElevatorConstants.goollL3),RobotStates.L3
        );
        addFunctionToOnChangeMap(
                ()->controller().setPosition(ElevatorConstants.goollL4),RobotStates.L4
        );
        addFunctionToOnChangeMap(
                ()->controller().setPosition(ElevatorConstants.goollclose),RobotStates.CLOSE,RobotStates.RESET
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
