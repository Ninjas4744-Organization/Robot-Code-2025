package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class HopperSushiConstans {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 40;
        kControllerConstants.subsystemName = "HopperSushi";

        kControllerConstants.encoderConversionFactor = 0.12345;
        kControllerConstants.controlConstants = ControlConstants.createPID(0.1, 0, 0, 0);
        kControllerConstants.positionGoalTolerance = 10;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }
    public static final double kIntakeState = 0.7;
    public static final double kCloseState = 0;
}
