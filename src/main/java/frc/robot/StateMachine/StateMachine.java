package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

public class StateMachine extends StateMachineIO<RobotStates> {
    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        switch (currentState) {
            case SHOOT_SPEAKER_READY:
                if (wantedState == RobotStates.SHOOT
                  || wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET) return true;
                break;

            case SHOOT_AMP_READY:
                if (wantedState == RobotStates.SHOOT
                  || wantedState == RobotStates.SHOOT_AMP_PREPARE
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET) return true;
                break;

            case CLIMB_READY:
                if (wantedState == RobotStates.CLIMB
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET) return true;
                break;

            case CLOSE, RESET:
                if (wantedState == RobotStates.IDLE) return true;
                break;

            case IDLE:
                if (wantedState == RobotStates.NOTE_IN_INDEXER
                  || wantedState == RobotStates.NOTE_SEARCH
                  || wantedState == RobotStates.RESET
                  || wantedState == RobotStates.CLOSE) return true;
                break;

            case SHOOT:
                if (wantedState == RobotStates.RESET || wantedState == RobotStates.CLOSE)
                    return true;
                break;

            case INTAKE:
                if (wantedState == RobotStates.RESET
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.NOTE_IN_INDEXER
                  || wantedState == RobotStates.INDEX
                  || wantedState == RobotStates.OUTTAKE) return true;
                break;

            case CLIMB:
                if (wantedState == RobotStates.CLIMBED) return true;
                break;

            case CLIMBED:
                if (wantedState == RobotStates.CLIMB_PREPARE) return true;
                break;

            case CLIMB_PREPARE:
                if (wantedState == RobotStates.RESET
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.CLIMB_READY) return true;
                break;

            case SHOOT_SPEAKER_PREPARE:
                if (wantedState == RobotStates.RESET
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.SHOOT_SPEAKER_READY) return true;
                break;

            case SHOOT_AMP_PREPARE:
                if (wantedState == RobotStates.RESET
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.SHOOT_AMP_READY) return true;
                break;

            case NOTE_SEARCH:
                if (wantedState == RobotStates.INTAKE
                  || wantedState == RobotStates.CLIMB_PREPARE
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
                  || wantedState == RobotStates.OUTTAKE
                  || wantedState == RobotStates.RESET
                  || wantedState == RobotStates.DRIVE_TO_AMP) return true;
                break;

            case NOTE_IN_INDEXER:
                if (wantedState == RobotStates.DRIVE_TO_AMP
                  || wantedState == RobotStates.DRIVE_TO_SOURCE
                  || wantedState == RobotStates.CLIMB_PREPARE
                  || wantedState == RobotStates.SHOOT_AMP_PREPARE
                  || wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.DELIVERY
                  || wantedState == RobotStates.RESET
                  || wantedState == RobotStates.OUTTAKE
                  || wantedState == RobotStates.OOGA_BOOGA) return true;
                break;

            case DRIVE_TO_AMP:
                if (wantedState == RobotStates.SHOOT_AMP_PREPARE
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET) return true;
                break;

            case DRIVE_TO_SOURCE:
                if (wantedState == RobotStates.INTAKE
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET) return true;
                break;

            case INDEX:
                if (wantedState == RobotStates.INDEX_BACK
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET
                  || wantedState == RobotStates.OUTTAKE) return true;
                break;

            case INDEX_BACK:
                if (wantedState == RobotStates.NOTE_IN_INDEXER
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET
                  || wantedState == RobotStates.OUTTAKE) return true;
                break;

            case TESTING:
                if (wantedState == RobotStates.RESET) return true;
                break;

            case OUTTAKE:
                if (wantedState == RobotStates.CLOSE || wantedState == RobotStates.RESET)
                    return true;
                break;

            case DELIVERY:
                if (wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET
                  || wantedState == RobotStates.SHOOT) return true;
                break;

            case OOGA_BOOGA:
                if (wantedState == RobotStates.OOGA_BOOGA_READY
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.RESET) return true;
                break;

            case OOGA_BOOGA_READY:
                if (wantedState == RobotStates.OOGA_BOOGA
                  || wantedState == RobotStates.CLOSE
                  || wantedState == RobotStates.SHOOT
                  || wantedState == RobotStates.RESET) return true;
                break;
        }
        return false;
    }

    private Timer _shootTimer;
    private Timer _outtakeTimer;
    @Override
    protected void setEndConditionMap() {
        addEndCondition(
          RobotStates.RESET,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().isResetted()
              && Indexer.getInstance().isResetted()
              && Shooter.getInstance().isResetted(),
            RobotStates.IDLE));

        addEndCondition(
          RobotStates.CLOSE,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().isHomed()
              && Indexer.getInstance().isResetted()
              && Shooter.getInstance().isResetted(),
            RobotStates.IDLE));

        addEndCondition(
          RobotStates.INTAKE, new StateEndCondition<>(RobotState.getInstance()::getNoteInIndexer, RobotStates.INDEX));

        addEndCondition(
          RobotStates.INDEX, new StateEndCondition<>(() -> !RobotState.getInstance().getNoteInIndexer(), RobotStates.INDEX_BACK));

        addEndCondition(
          RobotStates.INDEX_BACK,
          new StateEndCondition<>(RobotState.getInstance()::getNoteInIndexer, RobotStates.NOTE_IN_INDEXER));

        addEndCondition(
          RobotStates.OUTTAKE, new StateEndCondition<>(() -> _outtakeTimer.get() > 1, RobotStates.CLOSE));

        addEndCondition(
          RobotStates.SHOOT_AMP_PREPARE,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().atGoal()
              && Shooter.getInstance().isReady(),
            RobotStates.SHOOT_AMP_READY));

        addEndCondition(
          RobotStates.SHOOT_SPEAKER_PREPARE,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().atGoal()
              && Shooter.getInstance().isReady(),
            RobotStates.SHOOT_SPEAKER_READY));

        addEndCondition(
          RobotStates.OOGA_BOOGA,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().atGoal()
              && Shooter.getInstance().isReady(),
            RobotStates.OOGA_BOOGA_READY));

        addEndCondition(
          RobotStates.OOGA_BOOGA_READY,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().atGoal()
              && Shooter.getInstance().isReady(),
            RobotStates.OOGA_BOOGA));

        addEndCondition(
          RobotStates.DELIVERY,
          new StateEndCondition<>(
            () -> ShooterAngle.getInstance().atGoal()
              && Shooter.getInstance().isReady(),
            RobotStates.SHOOT));

        addEndCondition(RobotStates.SHOOT_AMP_READY, new StateEndCondition<>(() -> true, RobotStates.SHOOT));

        addEndCondition(
          RobotStates.SHOOT_SPEAKER_READY,
          new StateEndCondition<>(
            () -> !ShooterAngle.getInstance().atGoal()
              || !Shooter.getInstance().isReady(),
            RobotStates.SHOOT_SPEAKER_PREPARE));

        addEndCondition(RobotStates.SHOOT, new StateEndCondition<>(() -> _shootTimer.get() > 1, RobotStates.CLOSE));

//		addEndCondition(
//				RobotStates.DRIVE_TO_AMP,
//				new StateEndCondition<>(
//						() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.SHOOT_AMP_PREPARE));
//
//		addEndCondition(
//				RobotStates.DRIVE_TO_SOURCE,
//				new StateEndCondition<>(() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.INTAKE));

        addEndCondition(RobotStates.IDLE, new StateEndCondition<>(() -> RobotState.getInstance().getNoteInIndexer(), RobotStates.NOTE_IN_INDEXER));
        addEndCondition(RobotStates.IDLE, new StateEndCondition<>(() -> !RobotState.getInstance().getNoteInIndexer(), RobotStates.NOTE_SEARCH));
    }

    @Override
    protected void setFunctionMaps() {
        _shootTimer = new Timer();
        _outtakeTimer = new Timer();

        addFunctionToOnChangeMap(_shootTimer::restart, RobotStates.SHOOT);
        addFunctionToOnChangeMap(_outtakeTimer::restart, RobotStates.OUTTAKE);
        addFunctionToOnChangeMap(() ->
            RobotState.getInstance().setRobotState(
              RobotState.getInstance().getNoteInIndexer() ? RobotStates.NOTE_IN_INDEXER : RobotStates.NOTE_SEARCH),
          RobotStates.IDLE);
    }
}
