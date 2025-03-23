package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.util.Units;

public class OuttakeAngleConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 31;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "OuttakeAngle";

        kControllerConstants.encoderConversionFactor = 18 / 100.0 * Math.PI * 2;
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(10, 0, 0, 0, 10, 10, 0, 0, 0, 1, GravityTypeValue.Arm_Cosine);
        kControllerConstants.positionGoalTolerance = Units.degreesToRadians(10);
        kControllerConstants.encoderHomePosition = Units.degreesToRadians(135);
        kControllerConstants.isLimitSwitch = true;
        kControllerConstants.limitSwitchID = 3;
        kControllerConstants.limitSwitchDirection = 1;
        kControllerConstants.limitSwitchAutoStopReset = false;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kCoralState = Units.degreesToRadians(135);
    public static final double kAlgaeState = Units.degreesToRadians(35);
    public static final double kL1State = Units.degreesToRadians(-7.5);//125 / 360.0;//0;
    public static final double kClimbState = Units.degreesToRadians(-5);//125 / 360.0;//0;
    public static final double kResetSpeed = 0.1;
}
