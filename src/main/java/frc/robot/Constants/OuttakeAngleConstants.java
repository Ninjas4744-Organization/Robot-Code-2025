package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class OuttakeAngleConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 31;
        kControllerConstants.subsystemName = "OuttakeAngle";

        kControllerConstants.encoderConversionFactor = 18 / 100.0 * 360;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(0, 0, 0, 0, 0, 0, 0, 0);
        kControllerConstants.positionGoalTolerance = 3;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final int kCoralState = 180;
    public static final int kAlgaeState = 0;
    public static final int kL1State = 0;
    public static final int kLimitSwitchID = 1;
}
