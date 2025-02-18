package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class OuttakeConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 30;
        kControllerConstants.subsystemName = "Outtake";

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kOuttakeState = 1;
    public static final double kIntakeState = 0.7;
    public static final double kRemoveAlgae = -1;
    public static final double kCloseState = 0;

    public static final double kOuttakeTime = 1;
}
