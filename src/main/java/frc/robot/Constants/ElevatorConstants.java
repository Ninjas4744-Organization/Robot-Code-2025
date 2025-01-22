package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.ControlConstants;
import com.ninjas4744.NinjasLib.DataClasses.ControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.MainControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SimulatedControllerConstants;

public class ElevatorConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants =
            new SimulatedControllerConstants();

    static {
        kControllerConstants.main.id=21;
        kControllerConstants.main.inverted=false;
        kControllerConstants.currentLimit=50;
        kControllerConstants.subsystemName="Elevator";
        kControllerConstants.controlConstants = ControlConstants.createProfiledPID(0, 0,0, 5,6,6,0,0);
        kControllerConstants.followers= new ControllerConstants[0];
        kControllerConstants.followers[0].id=22;
        kControllerConstants.followers[0].inverted=true;
        kControllerConstants.positionGoalTolerance=0;
        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorType= SimulatedControllerConstants.MotorType.KRAKEN;



    }
    public static final int kLimitSwitchID=4;
    public static final double kL1State=2;
    public static final double kL2state=4;
    public static final double kL3State=6;
    public static final double kL4State=10;
    public static final double kCloseState=0;


}
