package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;

public class StateMachine extends StateMachineIO<RobotStates> {
    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        return true;
    }

    @Override
    protected void setEndConditionMap() {
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
