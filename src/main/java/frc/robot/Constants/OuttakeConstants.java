package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class OuttakeConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 30;
        kControllerConstants.subsystemName = "Subsystems/Outtake";
        kControllerConstants.currentLimit = 70;

        kControllerConstants.controlConstants = ControlConstants.createTorqueCurrent(9, 0.2);

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kL4OuttakeState = 80;
    public static final double kOuttakeState = 17;
    public static final double kL1OuttakeState = -10;
    public static final double kIntakeState = 15;
    public static final double kIndexBackState = -5;
    public static final double kOuttakeBackState = -6.5;
    public static final double kIndexState = 7.5;
    public static final double kRemoveAlgae = -100;
}
