package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.StateMachineIO;

public class StateMachine extends StateMachineIO<RobotStates> {
    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        return false;
    }

    @Override
    protected void setEndConditionMap() {

    }

    @Override
    protected void setFunctionMaps() {

    }
}
