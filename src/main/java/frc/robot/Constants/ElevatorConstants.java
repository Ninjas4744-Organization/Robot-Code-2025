package frc.robot.Constants;

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
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(2, 0, 0, 0, 65, 130, 0, 0.29);
        kControllerConstants.positionGoalTolerance = 0.01;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final int kLimitSwitchID = 0;
    public static final double[] kLStates = new double[]{
            0.1,
            0.5,
            0.9,
            1.3
    };
    public static final double kCloseState = 0;
    public static final double kResetSpeed = -0.15;
}
