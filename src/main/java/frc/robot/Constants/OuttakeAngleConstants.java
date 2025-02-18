package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class OuttakeAngleConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 41;
        kControllerConstants.subsystemName = "OuttakeAngle";

        kControllerConstants.encoderConversionFactor = 0.25 * 0.05 * Math.PI;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(1, 0, 0, 0, 2, 4, 0, 0.1);
        kControllerConstants.positionGoalTolerance = 0.05;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final int kCoralState = 180;
    public static final int kAlgeaState = 0;
    public static final int kLimitSwitchID = 1;
}
