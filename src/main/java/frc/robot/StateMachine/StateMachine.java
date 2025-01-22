package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import frc.robot.Subsystems.Elevator;

import java.util.Timer;

public class StateMachine extends StateMachineIO<RobotStates> {
    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }
    private Timer _outtakeTimer;


    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        switch (currentState) {
            case IDLE:
                if (wantedState == RobotStates.CORAL_SEARCH)
                    return true;
                break;
            case CORAL_SEARCH:
                if (wantedState == RobotStates.INTAKE)
                    return true;
                break;
            case INTAKE:
                if (wantedState == RobotStates.CORAL_READY)
                    return true;
                break;
            case CORAL_READY:
                if (wantedState == RobotStates.L1 ||
                        wantedState == RobotStates.L2 ||
                        wantedState == RobotStates.L3 ||
                        wantedState == RobotStates.L4 ||
                        wantedState == RobotStates.REMOVE_ALGAE )
                    return true;
                break;
            case L1, L2, L3, L4, REMOVE_ALGAE:
                if (wantedState == RobotStates.GO_RIGHT_REEF ||
                        wantedState == RobotStates.GO_LEFT_REEF ||
                        wantedState == RobotStates.AT_CENTER_REEF)//* cak
                    return true;
                break;
            case GO_RIGHT_REEF,GO_LEFT_REEF:
                if (wantedState == RobotStates.OUTTAKE_READY)
                    return true;
                break;
            case OUTTAKE_READY:
                if (wantedState == RobotStates.OUTTAKE)
                    return true;
                break;
            case OUTTAKE:
                if (wantedState == RobotStates.CLOSE)
                    return true;
                break;
            case CLOSE:
                if (wantedState == RobotStates.IDLE)
                    return true;
                break;
            default:
                if (wantedState == RobotStates.RESET)
                    return true;
        }
        return false;
    }

    @Override
    protected void setEndConditionMap() {
        addEndCondition(RobotStates.OUTTAKE_READY, new StateEndCondition<>(()-> Elevator.getInstance().atGoal(), RobotStates.OUTTAKE));
//        addEndCondition(RobotStates.OUTTAKE, new StateEndCondition<>(()->_outtakeTimer<1 , RobotStates.CLOSE));
        addEndCondition(RobotStates.CLOSE, new StateEndCondition<>(()-> Elevator.getInstance().isResetted(), RobotStates.IDLE));
        addEndCondition(RobotStates.INTAKE, new StateEndCondition<>(()-> RobotState.getInstance().isintakFinish(), RobotStates.CORAL_READY));

        addEndCondition(RobotStates.L1, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.L2, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.L3, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.L4, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.GO_RIGHT_REEF, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.OUTTAKE_READY));
        addEndCondition(RobotStates.GO_LEFT_REEF, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.OUTTAKE_READY));
    }

    @Override
    protected void setFunctionMaps() {

    }

}
