package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import org.littletonrobotics.junction.Logger;

public class Elevator extends StateMachineMotoredSubsystem<RobotStates> {
    private static Elevator _instance;

    public static Elevator getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Elevator(paused);
    }

    private DigitalInput _limit;

    private Elevator(boolean paused){
        super(paused);

        if(!_paused)
            _limit = new DigitalInput(ElevatorConstants.kLimitSwitchID);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(ElevatorConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);
    }

    private boolean getLimit(){
        return !_limit.get();
    }

    @Override
    protected void resetSubsystemO() {
        runMotor(ElevatorConstants.kResetSpeed).until(this::getLimit).schedule();
    }

    @Override
    protected boolean isResettedO() {
        return !_limit.get();
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToPeriodicMap(() ->
        {
            if(SwerveSubsystem.getInstance().atPidingZone())
                controller().setPosition(ElevatorConstants.kLStates[RobotState.getInstance().getReefLevel() - 1]);
        },
        RobotStates.AT_SIDE_REEF, RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);

        addFunctionToPeriodicMap(() -> controller().setPosition(FieldConstants.getAlgaeLevel() == 1 ? ElevatorConstants.kRemoveAlgae : ElevatorConstants.kRemoveAlgae2), RobotStates.REMOVE_ALGAE);

        addFunctionToOnChangeMap(this::resetSubsystemO, RobotStates.RESET);
        addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CORAL_SEARCH, RobotStates.CORAL_READY);
        addFunctionToOnChangeMap(() -> Commands.run(() -> controller().setPosition(0)).until(() -> controller().atGoal()).andThen(runMotor(ElevatorConstants.kResetSpeed)).until(this::getLimit).schedule(), RobotStates.CLOSE);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_paused)
            return;

        Logger.recordOutput("Elevator Limit", getLimit());

        double stage2Height = controller().getPosition();
        double stage1Height = stage2Height >= 0.75 ? (stage2Height - 0.75) : 0;
        Logger.recordOutput("Robot Pose Elevator 1", new Pose3d(0, 0, stage1Height, new Rotation3d(Math.PI / 2, 0, Math.PI * 1.5)));
        Logger.recordOutput("Robot Pose Elevator 2", new Pose3d(0, 0, stage2Height, new Rotation3d(Math.PI / 2, 0, Math.PI * 1.5)));

        if (!RobotState.isSimulated() && getLimit()){
            controller().resetEncoder();
//            if (controller().getOutput() < 0)
//                controller().stop();
        }
    }
}
