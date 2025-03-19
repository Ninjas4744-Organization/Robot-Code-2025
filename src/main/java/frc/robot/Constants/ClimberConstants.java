package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;
import edu.wpi.first.math.util.Units;

public class ClimberConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();
    static {
        kControllerConstants.main.id = 50;
        kControllerConstants.main.inverted = true;
        kControllerConstants.subsystemName = "Climber";

        kControllerConstants.controlConstants = ControlConstants.createPID(64, 0, 0, 0);
        kControllerConstants.controlConstants.GravityType = GravityTypeValue.Arm_Cosine;
        kControllerConstants.positionGoalTolerance = Units.degreesToRadians(5);

        kControllerConstants.encoderConversionFactor = 1 / 125.0 * Math.PI * 2 / 2.12;
        kControllerConstants.encoderHomePosition = Units.degreesToRadians(-90);

        kControllerConstants.isLimitSwitch = true;
        kControllerConstants.limitSwitchInverted = true;
        kControllerConstants.limitSwitchDirection = 1;
        kControllerConstants.limitSwitchID = 5;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
    }

    public static final double kStage1 = Units.degreesToRadians(80);
}
