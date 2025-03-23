package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ElevatorConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 20;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "Elevator";

        kControllerConstants.followers = new MainControllerConstants.ControllerConstants[1];
        kControllerConstants.followers[0] = new MainControllerConstants.ControllerConstants();
        kControllerConstants.followers[0].id = 21;
        kControllerConstants.followers[0].inverted = false;

        kControllerConstants.encoderConversionFactor = 0.25 * 0.05 * Math.PI;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 5, 7, 100, 0, 0.47, 0.4, GravityTypeValue.Elevator_Static);
        kControllerConstants.positionGoalTolerance = 0.03;
        kControllerConstants.isMaxSoftLimit = true;
        kControllerConstants.maxSoftLimit = 1.12;
        kControllerConstants.isLimitSwitch = true;
        kControllerConstants.limitSwitchID = 2;
        kControllerConstants.limitSwitchInverted = true;
        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double[] kLStates = new double[]{
            0,
            0.13,
            0.555,
            //1.09//sadna
            1.13//comp
    };

    public static final double kRemoveAlgae = 0.4-0.03-0.03-0.01;
    public static final double kRemoveAlgae2 = 0.7;
    public static final double kResetSpeed = -0.2;
}
