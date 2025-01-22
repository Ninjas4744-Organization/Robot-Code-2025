package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import frc.robot.Subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class StateMachine extends StateMachineIO<RobotStates> {
    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }

    private Timer _outtakeTimer;

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        switch (currentState) {
            case IDLE:
                return wantedState == RobotStates.CORAL_SEARCH;
            case CORAL_SEARCH:
                return wantedState == RobotStates.INTAKE || wantedState ==RobotStates.REMOVE_ALGAE;
            case INTAKE:
                return wantedState == RobotStates.CORAL_READY;
            case CORAL_READY:
                return wantedState == RobotStates.L1 ||
                        wantedState == RobotStates.L2 ||
                        wantedState == RobotStates.L3 ||
                        wantedState == RobotStates.L4 ||
                        wantedState == RobotStates.REMOVE_ALGAE ||
                        wantedState==RobotStates.OUTTAKE_READY;
            case L1, L2, L3, L4:
                return wantedState == RobotStates.GO_RIGHT_REEF ||
                        wantedState == RobotStates.GO_LEFT_REEF;
            case GO_RIGHT_REEF,GO_LEFT_REEF:
                return wantedState == RobotStates.OUTTAKE_READY;
            case OUTTAKE_READY:
                return wantedState == RobotStates.OUTTAKE;
            case OUTTAKE:
                return wantedState == RobotStates.CLOSE;
            case CLOSE, REMOVE_ALGAE:
                return wantedState == RobotStates.IDLE;
            default:
                if (wantedState == RobotStates.RESET)
                    return true;
        }
        return false;
    }

    @Override
    protected void setEndConditionMap() {
        addEndCondition(RobotStates.IDLE, new StateEndCondition<>(()-> true, RobotStates.CORAL_SEARCH));
        addEndCondition(RobotStates.INTAKE, new StateEndCondition<>(()-> RobotState.getInstance().isintakFinish(), RobotStates.CORAL_READY));
        addEndCondition(RobotStates.L1, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.L2, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.L3, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.L4, new StateEndCondition<>(() -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.GO_RIGHT_REEF));
        addEndCondition(RobotStates.GO_RIGHT_REEF, new StateEndCondition<>(() -> SwerveSubsystem.getInstance().atReefSide() , RobotStates.OUTTAKE_READY));
        addEndCondition(RobotStates.GO_LEFT_REEF, new StateEndCondition<>(() -> SwerveSubsystem.getInstance().atReefSide(), RobotStates.OUTTAKE_READY));
        addEndCondition(RobotStates.OUTTAKE_READY, new StateEndCondition<>(()-> true, RobotStates.CORAL_SEARCH));
    }

    @Override
    protected void setFunctionMaps() {

    }
}