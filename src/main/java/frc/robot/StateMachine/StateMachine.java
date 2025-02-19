package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import frc.robot.Constants.OuttakeConstants;
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
//    private Timer _hornTimer;
//    private Timer _coralTimer;

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        return switch (currentState) {
            case RESET, CLOSE -> wantedState == RobotStates.IDLE;

            case IDLE -> wantedState == RobotStates.CORAL_SEARCH
                    || wantedState == RobotStates.CORAL_READY;

            case CORAL_SEARCH -> wantedState == RobotStates.INTAKE
                    || wantedState == RobotStates.REMOVE_ALGAE
                    || wantedState == RobotStates.RESET;

            case INTAKE -> wantedState == RobotStates.CORAL_READY
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case CORAL_READY -> wantedState == RobotStates.GO_LEFT_REEF
                    || wantedState == RobotStates.GO_RIGHT_REEF
                    || wantedState == RobotStates.REMOVE_ALGAE
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

//            case L1, L2, L3, L4 -> wantedState == RobotStates.AT_CENTER_REEF
//                    || wantedState == RobotStates.RESET
//                    || wantedState == RobotStates.CLOSE;

//            case AT_CENTER_REEF -> wantedState == RobotStates.GO_RIGHT_REEF
//                    || wantedState == RobotStates.GO_LEFT_REEF
//                    || wantedState == RobotStates.RESET
//                    || wantedState == RobotStates.CLOSE;

            case REMOVE_ALGAE -> wantedState == RobotStates.RESET
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

            case OUTTAKE -> wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case TEST -> false;
        };
    }

    @Override
    protected void setEndConditionMap() {
        addEndCondition(RobotStates.INTAKE, new StateEndCondition<>(
                () -> RobotState.getInstance().isCoralInRobot(), RobotStates.CORAL_READY));

        /* Object Detection */
//        addEndCondition(RobotStates.L1, new StateEndCondition<>(
//                () -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.AT_CENTER_REEF));
//
//        addEndCondition(RobotStates.L2, new StateEndCondition<>(
//                () -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.AT_CENTER_REEF));
//
//        addEndCondition(RobotStates.L3, new StateEndCondition<>(
//              () -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.AT_CENTER_REEF));
//
//        addEndCondition(RobotStates.L4, new StateEndCondition<>(
//              () -> SwerveController.getInstance().isDriveAssistFinished(), RobotStates.AT_CENTER_REEF));

//        addEndCondition(RobotStates.AT_CENTER_REEF, new StateEndCondition<>(
//               () -> _coralTimer.get() > CoralDetectionConstants.kDetectionTime && (CoralObjectDetection.getCoralDetection() == DetectedCoral.LEFT || CoralObjectDetection.getCoralDetection() == DetectedCoral.NONE), RobotStates.GO_RIGHT_REEF));
//
//        addEndCondition(RobotStates.AT_CENTER_REEF, new StateEndCondition<>(
//              () -> _coralTimer.get() > CoralDetectionConstants.kDetectionTime && CoralObjectDetection.getCoralDetection() == DetectedCoral.RIGHT, RobotStates.GO_LEFT_REEF));
//
//        addEndCondition(RobotStates.AT_CENTER_REEF, new StateEndCondition<>(
//             () -> _coralTimer.get() > CoralDetectionConstants.kDetectionTime && CoralObjectDetection.getCoralDetection() == DetectedCoral.BOTH, RobotStates.CORAL_READY));
        /* /Object Detection */

        addEndCondition(RobotStates.GO_RIGHT_REEF, new StateEndCondition<>(
                () -> SwerveSubsystem.getInstance().atReefSide(), RobotStates.AT_SIDE_REEF));

        addEndCondition(RobotStates.GO_LEFT_REEF, new StateEndCondition<>(
                () -> SwerveSubsystem.getInstance().atReefSide(), RobotStates.AT_SIDE_REEF));

        addEndCondition(RobotStates.AT_SIDE_REEF, new StateEndCondition<>(
                () -> Elevator.getInstance().atGoal() && OuttakeAngle.getInstance().atGoal(), RobotStates.OUTTAKE_READY));

        addEndCondition(RobotStates.OUTTAKE_READY, new StateEndCondition<>(
                () -> true, RobotStates.OUTTAKE));

        addEndCondition(RobotStates.OUTTAKE, new StateEndCondition<>(
                () -> _outtakeTimer.get() > OuttakeConstants.kOuttakeTime, RobotStates.CLOSE));

//        addEndCondition(RobotStates.REMOVE_ALGAE, new StateEndCondition<>(
//                () -> _hornTimer.get() > HornConstants.kRemoveAlgaeTime, RobotStates.CLOSE));

        addEndCondition(RobotStates.CLOSE, new StateEndCondition<>(
                () -> Elevator.getInstance().isResetted()
                    && Sushi.getInstance().isResetted()
                    && OuttakeAngle.getInstance().isResetted()
                    && Outtake.getInstance().isResetted(), RobotStates.IDLE));

        addEndCondition(RobotStates.RESET, new StateEndCondition<>(
                () -> Elevator.getInstance().isResetted()
                    && Sushi.getInstance().isResetted()
                    && OuttakeAngle.getInstance().isResetted()
                    && Outtake.getInstance().isResetted(), RobotStates.IDLE));
    }

    @Override
    protected void setFunctionMaps() {
        _outtakeTimer = new Timer();
//        _hornTimer = new Timer();
//        _coralTimer = new Timer();

        addFunctionToOnChangeMap(_outtakeTimer::restart, RobotStates.OUTTAKE);
//        addFunctionToOnChangeMap(_hornTimer::restart, RobotStates.REMOVE_ALGAE);
//        addFunctionToOnChangeMap(_coralTimer::restart, RobotStates.AT_CENTER_REEF);//?
    }

    @Override
    public void periodic() {
        super.periodic();

        if(RobotState.getInstance().getRobotState() == RobotStates.IDLE)
            changeRobotState(RobotState.getInstance().isCoralInRobot() ? RobotStates.CORAL_READY : RobotStates.CORAL_SEARCH);
    }
}
