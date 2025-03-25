package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import com.ninjas4744.NinjasLib.Swerve.SwerveIO;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Elevator extends StateMachineMotoredSubsystem<RobotStates> {
    private static Elevator _instance;

    public static Elevator getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Elevator(paused);
    }

    private Elevator(boolean paused){
        super(paused);
    }

    @Override
    protected void setController() {
        _controller = new NinjasTalonFXController(ElevatorConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);
    }
    @Override
    protected void resetSubsystemO() {
        runMotor(ElevatorConstants.kResetSpeed).until(_controller::getLimit).schedule();
    }

    @Override
    protected boolean isResettedO() {
        return controller().getLimit();
    }

    @Override
    protected void setFunctionMaps() {

    }

    public Command setPosition(DoubleSupplier position){
        return Commands.runOnce(() -> controller().setPosition(position.getAsDouble()));
    }

    public Command setPositionWhenSpeedBelow(DoubleSupplier position, DoubleSupplier robotSpeed){
        return Commands.waitUntil(() -> {
            double speed = new Translation2d(
                    SwerveIO.getInstance().getChassisSpeeds(false).vxMetersPerSecond,
                    SwerveIO.getInstance().getChassisSpeeds(false).vyMetersPerSecond).getNorm();

            return speed < robotSpeed.getAsDouble();
        }).andThen(setPosition(position));
    }

    public Command close(){
        return Commands.run(() -> controller().setPosition(0))
                .until(() -> controller().getPosition() < 0.15)
                .andThen(runMotor(ElevatorConstants.kResetSpeed)).until(controller()::getLimit);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(_paused)
            return;

        double stage2Height = controller().getPosition();
        double stage1Height = stage2Height >= 0.75 ? (stage2Height - 0.75) : 0;
        Logger.recordOutput("Other/Robot Pose Elevator 1", new Pose3d(0, 0, stage1Height, new Rotation3d(Math.PI / 2, 0, Math.PI * 1.5)));
        Logger.recordOutput("Other/Robot Pose Elevator 2", new Pose3d(0, 0, stage2Height, new Rotation3d(Math.PI / 2, 0, Math.PI * 1.5)));
    }
}
