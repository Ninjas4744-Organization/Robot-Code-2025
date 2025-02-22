package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class OuttakeAngleConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 31;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "OuttakeAngle";

        kControllerConstants.encoderConversionFactor = 18 / 100.0 * 360;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(4 / (18 / 100.0 * 360), 0, 0.2 / (18 / 100.0 * 360), 0, 30 / (18 / 100.0 * 360), 30 / (18 / 100.0 * 360), 0, 0.125, 0);
        kControllerConstants.positionGoalTolerance = 5;
        kControllerConstants.encoderHomePosition = 180;
        kControllerConstants.isMaxSoftLimit = true;
        kControllerConstants.maxSoftLimit = 15;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final int kCoralState = 180;
    public static final int kAlgaeState = 75;
    public static final int kL1State = 0;
    public static final int kLimitSwitchID = 3;
}
