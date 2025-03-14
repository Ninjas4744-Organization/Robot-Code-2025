package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
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

        kControllerConstants.encoderConversionFactor = 18 / 100.0;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(75, 0, 0, 0, 2, 2, 0, 0, 0.2, 0.3);
        kControllerConstants.controlConstants.GravityType = GravityTypeValue.Arm_Cosine;
        kControllerConstants.positionGoalTolerance = 2 / 360.0;
        kControllerConstants.encoderHomePosition = 135 / 360.0;
        kControllerConstants.isMinSoftLimit = false;
        kControllerConstants.minSoftLimit = 15;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kCoralState = 135 / 360.0;
    public static final double kAlgaeState = 35 / 360.0;
    public static final double kL1State = 0;//135 / 360.0;//0;
    public static final int kLimitSwitchID = 3;
    public static final double kResetSpeed = 0.1;
}
