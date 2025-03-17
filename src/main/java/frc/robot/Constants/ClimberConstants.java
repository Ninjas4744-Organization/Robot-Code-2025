package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ClimberConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 50;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "Climber";
        kControllerConstants.encoderConversionFactor = 1 / 125.0;
        kControllerConstants.controlConstants = ControlConstants.createPID(64, 0, 0, 0);
        kControllerConstants.controlConstants.GravityType = GravityTypeValue.Arm_Cosine;
        kControllerConstants.positionGoalTolerance = 5 / 360.0;
        kControllerConstants.encoderHomePosition = 180 / 360.0;
        kControllerConstants.isLimitSwitch = true;
        kControllerConstants.limitSwitchInverted = true;
        kControllerConstants.limitSwitchDirection = 1;
        kControllerConstants.limitSwitchID = 5;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kCloseState = -90 / 360.0;
    public static final double kStage1 = 0 / 360.0;
    public static final double kStage2 = 180 / 360.0;
    public static final double kResetSpeed = -0.25;
}
