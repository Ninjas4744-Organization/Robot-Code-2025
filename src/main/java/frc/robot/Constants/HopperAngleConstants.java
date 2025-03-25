package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.util.Units;

public class HopperAngleConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 41;
        kControllerConstants.main.inverted = false;
        kControllerConstants.subsystemName = "Subsystems/HopperAngle";

        kControllerConstants.encoderConversionFactor = 18 / 100.0 / 16 * Math.PI * 2;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(10, 0, 0, 0, 2, 2, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        kControllerConstants.positionGoalTolerance = Units.degreesToRadians(2);
        kControllerConstants.encoderHomePosition = 0;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }
    public static final double kCloseState = 0;
    public static final double kOpenState = Units.degreesToRadians(213);
}
