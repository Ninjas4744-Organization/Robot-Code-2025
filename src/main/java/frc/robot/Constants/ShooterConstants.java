package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ShooterConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants =
        new SimulatedControllerConstants();

    static {
        kControllerConstants.main.id = 30;
        kControllerConstants.main.inverted = true;
        kControllerConstants.currentLimit = 40;
        kControllerConstants.subsystemName = "Shooter";

        kControllerConstants.controlConstants = ControlConstants.createTorqueCurrent(7.5, 0.185);
        kControllerConstants.velocityGoalTolerance = 600;
        kControllerConstants.encoderConversionFactor = 60;

        kControllerConstants.followers = new ControllerConstants[] {new ControllerConstants()};
        kControllerConstants.followers[0].id = 31;
        kControllerConstants.followers[0].inverted = true;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorTorque = 1;
    }

    public static final double kMinimumShootTolerance = 200;

    public enum States {
        kSpeaker(5580),
        kAmp(1625),
        kOuttake(1200),
        kDelivery(4600);

        private final double value;

        States(double value) {
            this.value = value;
        }

        public double get() {
            return value;
        }
    }
}