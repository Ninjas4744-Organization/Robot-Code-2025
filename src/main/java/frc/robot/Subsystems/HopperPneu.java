package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.HopperConstants;
import frc.robot.StateMachine.RobotStates;

public class HopperPneu extends StateMachineSubsystem<RobotStates> {
    private static HopperPneu _instance;

    public static HopperPneu getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new HopperPneu(paused);
    }

    private PneumaticHub _ph;
    private DoubleSolenoid _solenoid;

    public HopperPneu(boolean paused) {
        super(paused);

        if(!paused){
            _ph = new PneumaticHub(HopperConstants.kPneumaticID);
            _solenoid = _ph.makeDoubleSolenoid(HopperConstants.kForwardID, HopperConstants.kReverseID);
        }
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> _solenoid.set(DoubleSolenoid.Value.kForward), RobotStates.RESET);
    }
}
