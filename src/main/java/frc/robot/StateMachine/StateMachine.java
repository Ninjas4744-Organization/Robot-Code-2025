package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj.Timer;

public class StateMachine extends StateMachineIO<RobotStates> {
    public StateMachine(boolean paused) {
        super(paused);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }

    private Timer _outtakeTimer;
    private Timer _preOuttakeTimer;

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        return switch (currentState) {
            case RESET, CLOSE -> wantedState == RobotStates.IDLE
            || wantedState == RobotStates.RESET;

            case IDLE -> wantedState == RobotStates.CORAL_SEARCH
                    || wantedState == RobotStates.CORAL_READY;

            case CORAL_SEARCH -> wantedState == RobotStates.INTAKE
                    || wantedState == RobotStates.REMOVE_ALGAE
                    || wantedState == RobotStates.RESET;

            case INTAKE -> wantedState == RobotStates.INDEX_BACK
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case CORAL_READY -> wantedState == RobotStates.GO_LEFT_REEF
                    || wantedState == RobotStates.GO_RIGHT_REEF
                    || wantedState == RobotStates.AT_SIDE_REEF
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE
                    || wantedState == RobotStates.INTAKE;

            case INDEX_BACK -> wantedState == RobotStates.INDEX
                    || wantedState == RobotStates.CLOSE
                    || wantedState == RobotStates.RESET;

            case INDEX -> wantedState == RobotStates.CORAL_READY
                    || wantedState == RobotStates.IDLE
                    || wantedState == RobotStates.CLOSE
                    || wantedState == RobotStates.RESET;

            case GO_ALGAE -> wantedState == RobotStates.REMOVE_ALGAE
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case OUTTAKE, GO_ALGAE_BACK -> wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case REMOVE_ALGAE -> wantedState == RobotStates.GO_ALGAE_BACK
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case GO_RIGHT_REEF, GO_LEFT_REEF -> wantedState == RobotStates.AT_SIDE_REEF
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case AT_SIDE_REEF -> wantedState == RobotStates.OUTTAKE_READY
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case OUTTAKE_READY -> wantedState == RobotStates.OUTTAKE
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case TEST -> false;
        };
    }

    @Override
    protected void setEndConditionMap() {
        addEndCondition(RobotStates.INTAKE, new StateEndCondition<>(
                () -> RobotState.getInstance().isCoralInRobot(), RobotStates.INDEX_BACK));

        addEndCondition(RobotStates.INDEX_BACK, new StateEndCondition<>(
                () -> !RobotState.getInstance().isCoralInRobot(), RobotStates.INDEX));

        addEndCondition(RobotStates.INDEX, new StateEndCondition<>(
                () -> RobotState.getInstance().isCoralInRobot(), RobotStates.CORAL_READY));

        addEndCondition(RobotStates.GO_RIGHT_REEF, new StateEndCondition<>(
                () -> SwerveSubsystem.getInstance().atReef(), RobotStates.AT_SIDE_REEF));

        addEndCondition(RobotStates.GO_LEFT_REEF, new StateEndCondition<>(
                () -> SwerveSubsystem.getInstance().atReef(), RobotStates.AT_SIDE_REEF));

        addEndCondition(RobotStates.AT_SIDE_REEF, new StateEndCondition<>(
                () -> Elevator.getInstance().atGoal() && OuttakeAngle.getInstance().atGoal() && (SwerveSubsystem.getInstance().atReef() || RobotState.getInstance().getReefLevel() == 1), RobotStates.OUTTAKE_READY));

        addEndCondition(RobotStates.OUTTAKE_READY, new StateEndCondition<>(
                () -> RobotState.getInstance().getReefLevel() != 4 || _preOuttakeTimer.get() > 0.125, RobotStates.OUTTAKE));

        addEndCondition(RobotStates.OUTTAKE, new StateEndCondition<>(
                () -> !RobotState.getInstance().isCoralInRobot() && _outtakeTimer.get() > 0.2, RobotStates.CLOSE));

        addEndCondition(RobotStates.GO_ALGAE, new StateEndCondition<>(
                () -> SwerveSubsystem.getInstance().atReef(), RobotStates.REMOVE_ALGAE));

//        addEndCondition(RobotStates.REMOVE_ALGAE, new StateEndCondition<>(
//                () -> Elevator.getInstance().atGoal() && OuttakeAngle.getInstance().atGoal() && Outtake.getInstance().getCurrent() > 55, RobotStates.GO_ALGAE_BACK));

        addEndCondition(RobotStates.GO_ALGAE_BACK, new StateEndCondition<>(
                () -> SwerveSubsystem.getInstance().atReef(), RobotStates.CLOSE));

        addEndCondition(RobotStates.CLOSE, new StateEndCondition<>(
                () -> (Elevator.getInstance().isResetted())
                    && Sushi.getInstance().isResetted()
                    && OuttakeAngle.getInstance().isResetted()
                    && Outtake.getInstance().isResetted(), RobotStates.IDLE));

        addEndCondition(RobotStates.RESET, new StateEndCondition<>(
                () -> (Elevator.getInstance().isResetted())
                    && Sushi.getInstance().isResetted()
                    && OuttakeAngle.getInstance().isResetted()
                    && Outtake.getInstance().isResetted(), RobotStates.IDLE));
    }

    @Override
    protected void setFunctionMaps() {
        _outtakeTimer = new Timer();
        _preOuttakeTimer = new Timer();

        addFunctionToOnChangeMap(_outtakeTimer::restart, RobotStates.OUTTAKE);
        addFunctionToOnChangeMap(_preOuttakeTimer::restart, RobotStates.OUTTAKE_READY);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(RobotState.getInstance().getRobotState() == RobotStates.IDLE)
            changeRobotState(RobotState.getInstance().isCoralInRobot() ? RobotStates.CORAL_READY : RobotStates.CORAL_SEARCH);
    }
}
