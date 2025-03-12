package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ClimberConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 20;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "Climber";
        kControllerConstants.encoderConversionFactor = 0.25 * 0.05 * Math.PI;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 5, 7, 100, 0, 0.47, 0.4);
        kControllerConstants.controlConstants.GravityType = GravityTypeValue.Elevator_Static;
        kControllerConstants.positionGoalTolerance = 0.03;
        kControllerConstants.isMaxSoftLimit = true;
        kControllerConstants.maxSoftLimit = 1.12;
        kControllerConstants.isLimitSwitch = true;
        kControllerConstants.limitSwitchID = 4;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }
    public static final double kCloseState = 0;
    public static final double kOpenState = 5;
    public static final double kResetSpeed = -0.25;
}
