package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.ControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ElevatorConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 20;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "Elevator";

        kControllerConstants.followers = new ControllerConstants[1];
        kControllerConstants.followers[0] = new ControllerConstants();
        kControllerConstants.followers[0].id = 21;
        kControllerConstants.followers[0].inverted = false;

        kControllerConstants.encoderConversionFactor = 0.25 * 0.05 * Math.PI;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 2, 4, 0, 0.29, 0);
        kControllerConstants.positionGoalTolerance = 0.03;
        kControllerConstants.isMaxSoftLimit = true;
        kControllerConstants.maxSoftLimit = 1.12;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final int kLimitSwitchID = 2;
    public static final double[] kLStates = new double[]{
            0,
            0.18-0.025-0.01,
            0.5-0.02+0.015,
            1.12-0.015-0.01+0.01+0.01-0.01
    };
    public static final double kCloseState = 0;
    public static final double kRemoveAlgae = 0.4-0.03-0.03-0.01;
    public static final double kRemoveAlgae2 = 0.7;
    public static final double kResetSpeed = -0.25;
    public static final double kL1ResetSpeed = -0.25;
}
