package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class HornConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 40;
        kControllerConstants.subsystemName = "Horn";
        kControllerConstants.controlConstants = ControlConstants.createPID(0.1, 0, 0, 0);
        kControllerConstants.encoderConversionFactor = 0.12345;
        kControllerConstants.positionGoalTolerance = 10;
        kControllerConstants.isMaxSoftLimit = true;
        kControllerConstants.maxSoftLimit = 120;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kOpenState = 120;
}
