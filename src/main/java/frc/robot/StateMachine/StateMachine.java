package frc.robot.StateMachine;

import com.ninjas4744.NinjasLib.StateMachineIO;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommandBuilder;
import frc.robot.Constants.*;
import frc.robot.Subsystems.*;

public class StateMachine extends StateMachineIO<RobotStates> {
    public StateMachine(boolean paused) {
        super(paused);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineIO.getInstance();
    }

    @Override
    public boolean canChangeRobotState(RobotStates currentState, RobotStates wantedState) {
        if(wantedState == RobotStates.RESET || wantedState == RobotStates.TEST)
            return true;

        return switch (currentState) {
            case CLIMB1 -> wantedState == RobotStates.CLIMBED1 || wantedState == RobotStates.CLOSE;

            case CLIMBED1 -> wantedState == RobotStates.CLIMB2 || wantedState == RobotStates.CLOSE;

            case CLIMB2 -> wantedState == RobotStates.CLIMBED || wantedState == RobotStates.CLOSE;

            case CLIMBED, REMOVE_ALGAE -> wantedState == RobotStates.CLOSE;

            case OUTTAKE -> wantedState == RobotStates.CLOSE
                    || wantedState == RobotStates.REMOVE_ALGAE;

            case AT_REEF -> wantedState == RobotStates.OUTTAKE || wantedState == RobotStates.CLOSE;

            case RESET, CLOSE -> wantedState == RobotStates.IDLE;

            case IDLE -> wantedState == RobotStates.INTAKE
                    || wantedState == RobotStates.REMOVE_ALGAE
                    || wantedState == RobotStates.CORAL_READY
                    || wantedState == RobotStates.GO_REEF
                    || wantedState == RobotStates.CLIMB1;

            case INTAKE -> wantedState == RobotStates.CORAL_READY
                    || wantedState == RobotStates.CLOSE;

            case CORAL_READY -> wantedState == RobotStates.GO_REEF
                    || wantedState == RobotStates.AT_REEF
                    || wantedState == RobotStates.CLOSE
                    || wantedState == RobotStates.INTAKE
                    || wantedState == RobotStates.IDLE;

            case GO_REEF -> wantedState == RobotStates.AT_REEF
                    || wantedState == RobotStates.CLOSE;

            case TEST -> false;
        };
    }

    private int intakeCount = 0;
    @Override
    protected void setCommandMap() {
        addCommand(RobotStates.IDLE, Commands.run(() -> {
            if(RobotState.getInstance().isCoralInRobot())
                changeRobotState(RobotStates.CORAL_READY);

            if(RobotState.getInstance().getAlgae())
                changeRobotState(RobotStates.REMOVE_ALGAE);
        }).repeatedly());

        addCommand(RobotStates.RESET, Commands.sequence(
                Commands.runOnce(() -> {
                    RobotState.getInstance().setAlgae(false);
                    Elevator.getInstance().resetSubsystem();
                    OuttakeAngle.getInstance().resetSubsystem();
                    Outtake.getInstance().resetSubsystem();
                    Sushi.getInstance().resetSubsystem();
                    SwerveSubsystem.getInstance().resetSubsystem();
                    HopperAngle.getInstance().resetSubsystem();
                    Climber.getInstance().resetSubsystem();
                }),
                Commands.waitUntil(() -> Elevator.getInstance().isResetted() && OuttakeAngle.getInstance().isResetted()),
                CommandBuilder.changeRobotState(RobotStates.IDLE)
        ));

        addCommand(RobotStates.INTAKE, Commands.sequence(
                Sushi.getInstance().setPercent(SushiConstants.kIntake),
                Outtake.getInstance().intake().until(() -> RobotState.getInstance().isCoralInRobot()),

                Commands.sequence(
                        Outtake.getInstance().setVelocity(() -> OuttakeConstants.kIndexBackState),
                        Sushi.getInstance().setPercent(0),
                        Commands.waitUntil(() -> !RobotState.getInstance().isCoralInRobot()),
                        Commands.waitSeconds(0.045),
                        Outtake.getInstance().setVelocity(() -> OuttakeConstants.kIndexState),
                        Commands.waitUntil(() -> RobotState.getInstance().isCoralInRobot()),
                        Commands.runOnce(() -> intakeCount++)
                        ).repeatedly().until(() -> intakeCount == 1).finallyDo(() -> intakeCount = 0),

                CommandBuilder.changeRobotState(RobotStates.CLOSE)
        ));

        addCommand(RobotStates.CORAL_READY, Commands.run(() -> {
            if(!RobotState.getInstance().isCoralInRobot())
                changeRobotState(RobotStates.IDLE);
        }).repeatedly());

        addCommand(RobotStates.REMOVE_ALGAE, Commands.parallel(
                Outtake.getInstance().setVelocity(() -> OuttakeConstants.kRemoveAlgae),
                Elevator.getInstance().setPosition(() -> FieldConstants.getAlgaeLevel() == 1 ? ElevatorConstants.kRemoveAlgae : ElevatorConstants.kRemoveAlgae2),
                OuttakeAngle.getInstance().setPosition(() -> OuttakeAngleConstants.kAlgaeState)
        ));

        addCommand(RobotStates.GO_REEF, Commands.sequence(
                Commands.runOnce(() -> SwerveSubsystem.getInstance().goToReef().schedule()),
                Commands.waitUntil(() -> SwerveSubsystem.getInstance().atPidingZone()),
                Elevator.getInstance().setPosition(() -> ElevatorConstants.kLStates[!RobotState.isAutonomous() ? (RobotState.getInstance().getReefLevel() == 4 ? 2 : RobotState.getInstance().getReefLevel() - 1) : (RobotState.getInstance().getReefLevel() - 1)]),
                Commands.runOnce(() -> OuttakeAngle.getInstance().resetSubsystem()),
                Commands.waitUntil(() -> SwerveSubsystem.getInstance().atGoal()),
                CommandBuilder.changeRobotState(RobotStates.AT_REEF)
        ));

        addCommand(RobotStates.AT_REEF, Commands.sequence(
                Elevator.getInstance().setPosition(() -> ElevatorConstants.kLStates[RobotState.getInstance().getReefLevel() - 1]),
                OuttakeAngle.getInstance().setPosition(() -> RobotState.getInstance().getReefLevel() != 1 ? OuttakeAngleConstants.kCoralState : OuttakeAngleConstants.kL1State),
                Commands.waitUntil(() -> Elevator.getInstance().atGoal() && OuttakeAngle.getInstance().atGoal()),
                Commands.waitSeconds(RobotState.getInstance().getReefLevel() == 4 ? 0.125 : 0.09),
                CommandBuilder.changeRobotState(RobotStates.OUTTAKE)
        ));

        addCommand(RobotStates.OUTTAKE, Commands.sequence(
                Commands.runOnce(() -> SwerveSubsystem.getInstance().resetSubsystem()),
                Outtake.getInstance().outtake(),
                Commands.waitUntil(() -> !RobotState.getInstance().isCoralInRobot()),
                Commands.either(
                    Commands.waitSeconds(0.125),
                    Commands.waitSeconds(1),
                    () -> RobotState.getInstance().getReefLevel() != 1
                ),
                Commands.either(
                        CommandBuilder.changeRobotState(RobotStates.REMOVE_ALGAE),
                        CommandBuilder.changeRobotState(RobotStates.CLOSE),
                        () -> RobotState.getInstance().getAlgae()
                )
        ));

        addCommand(RobotStates.CLOSE, Commands.sequence(
                Commands.parallel(
                    Commands.runOnce(() -> RobotState.getInstance().setAlgae(false)),
                    Elevator.getInstance().close(),
                    Commands.runOnce(() -> OuttakeAngle.getInstance().resetSubsystem()),
                    Commands.runOnce(() -> {
                        Outtake.getInstance().resetSubsystem();
                        Sushi.getInstance().resetSubsystem();
                        SwerveSubsystem.getInstance().resetSubsystem();
                    })
                ),
                Commands.waitUntil(() -> Elevator.getInstance().isResetted() && OuttakeAngle.getInstance().isResetted()),
                CommandBuilder.changeRobotState(RobotStates.IDLE)
        ));

        addCommand(RobotStates.CLIMB1, Commands.sequence(
                OuttakeAngle.getInstance().setPosition(() -> OuttakeAngleConstants.kClimbState),
                Elevator.getInstance().close(),
                Commands.waitUntil(() -> OuttakeAngle.getInstance().atGoal()),
                OuttakeAngle.getInstance().stop(),

                Commands.parallel(
                    HopperAngle.getInstance().setPosition(HopperAngleConstants.kOpenState),
                    Climber.getInstance().stage1()
                ),
                Commands.waitUntil(() -> Climber.getInstance().atGoal() && HopperAngle.getInstance().atGoal()),
                CommandBuilder.changeRobotState(RobotStates.CLIMBED1)
        ));

        addCommand(RobotStates.CLIMB2, Commands.sequence(
//                Climber.getInstance().stage2(),
//                CommandBuilder.changeRobotState(RobotStates.CLIMBED)
        ));
    }

    @Override
    protected void setFunctionMaps() {

    }
}
