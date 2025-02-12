package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HornAngleConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class Hopper extends StateMachineSubsystem<RobotStates> {
    private static Hopper _instance;

    public static Hopper getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Hopper(paused);
    }

    private Servo _servo;

    public Hopper(boolean paused) {
        super(paused);

        if(!paused)
            _servo = new Servo(HopperConstants.kServoID);
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> _servo.set(1), RobotStates.RESET);
    }
}
