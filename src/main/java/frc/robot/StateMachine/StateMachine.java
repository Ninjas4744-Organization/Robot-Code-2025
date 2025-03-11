package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.DataClasses.StateEndCondition;
import com.ninjas4744.NinjasLib.StateMachineIO;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommandBuilder;
import frc.robot.Constants.*;
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
                    || wantedState == RobotStates.AT_REEF
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

            case GO_RIGHT_REEF, GO_LEFT_REEF -> wantedState == RobotStates.AT_REEF
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case AT_REEF -> wantedState == RobotStates.OUTTAKE_READY
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case OUTTAKE_READY -> wantedState == RobotStates.OUTTAKE
                    || wantedState == RobotStates.RESET
                    || wantedState == RobotStates.CLOSE;

            case TEST -> false;
        };
    }

    @Override
    protected void setCommandMap() {
        addCommand(RobotStates.IDLE, Commands.runOnce(() -> {
            if(RobotState.getInstance().isCoralInRobot())
                changeRobotState(RobotStates.CORAL_READY);
        }).repeatedly());

        addCommand(RobotStates.RESET, Commands.runOnce(() -> {
                Elevator.getInstance().resetSubsystem();
                OuttakeAngle.getInstance().resetSubsystem();
                Outtake.getInstance().resetSubsystem();
                Sushi.getInstance().resetSubsystem();
                SwerveSubsystem.getInstance().resetSubsystem();
        }).until(() -> Elevator.getInstance().isResetted() &&
                OuttakeAngle.getInstance().isResetted())
                .andThen(CommandBuilder.changeRobotState(RobotStates.IDLE)));

        addCommand(RobotStates.INTAKE, Commands.sequence(
                Outtake.getInstance().setVelocity(OuttakeConstants.kIntakeState),
                Sushi.getInstance().setPercent(SushiConstants.kIntake),
                Commands.waitUntil(() -> RobotState.getInstance().isCoralInRobot()),

                Outtake.getInstance().setVelocity(OuttakeConstants.kIndexBackState),
                Sushi.getInstance().setPercent(0),
                Commands.waitUntil(() -> !RobotState.getInstance().isCoralInRobot()),

                Outtake.getInstance().setVelocity(OuttakeConstants.kIndexState),
                Commands.waitUntil(() -> RobotState.getInstance().isCoralInRobot()),

                CommandBuilder.changeRobotState(RobotStates.CORAL_READY)
        ));

        addCommand(RobotStates.CORAL_READY, Commands.run(() -> {
            if(!RobotState.getInstance().isCoralInRobot())
                changeRobotState(RobotStates.IDLE);
        }).repeatedly());

        addCommand(RobotStates.REMOVE_ALGAE, Commands.parallel(
                Outtake.getInstance().setVelocity(OuttakeConstants.kRemoveAlgae),
                Elevator.getInstance().setPosition(FieldConstants.getAlgaeLevel() == 1 ? ElevatorConstants.kRemoveAlgae : ElevatorConstants.kRemoveAlgae2),
                OuttakeAngle.getInstance().setPosition(OuttakeAngleConstants.kAlgaeState)
        ));

        addCommand(RobotStates.GO_REEF, Commands.sequence(
                Elevator.getInstance().setPosition(ElevatorConstants.kLStates[RobotState.getInstance().getReefLevel() - 1]),
                OuttakeAngle.getInstance().setPosition(OuttakeAngleConstants.kCoralState),
                SwerveSubsystem.getInstance().goToReef(),
                Commands.waitUntil(() -> SwerveSubsystem.getInstance().atReef()),
                CommandBuilder.changeRobotState(RobotStates.AT_REEF)
        ));

        addCommand(RobotStates.AT_REEF, Commands.sequence(
                Elevator.getInstance().setPosition(ElevatorConstants.kLStates[RobotState.getInstance().getReefLevel() - 1]),
                OuttakeAngle.getInstance().setPosition(OuttakeAngleConstants.kCoralState),
                Commands.waitUntil(() -> Elevator.getInstance().atGoal() && OuttakeAngle.getInstance().atGoal()),
                Commands.waitSeconds(0.125),
                CommandBuilder.changeRobotState(RobotStates.OUTTAKE)
        ));

        addCommand(RobotStates.OUTTAKE, Commands.sequence(
                Outtake.getInstance().outtake(),
                Commands.waitUntil(() -> RobotState.getInstance().isCoralInRobot()),
                Commands.waitSeconds(0.2),
                CommandBuilder.changeRobotState(RobotStates.IDLE)
        ));

        addCommand(RobotStates.CLOSE, Commands.runOnce(() -> {
                    Elevator.getInstance().setPosition(0);
                    OuttakeAngle.getInstance().setPosition(OuttakeAngleConstants.kCoralState);
                    Outtake.getInstance().resetSubsystem();
                    Sushi.getInstance().resetSubsystem();
                    SwerveSubsystem.getInstance().resetSubsystem();
                }).until(() -> Elevator.getInstance().isResetted() &&
                        OuttakeAngle.getInstance().isResetted())
                .andThen(CommandBuilder.changeRobotState(RobotStates.IDLE)));
    }

    @Override
    protected void setFunctionMaps() {

    }
}
