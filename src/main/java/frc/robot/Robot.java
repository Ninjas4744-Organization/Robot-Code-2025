package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.ninjas4744.NinjasLib.Swerve.SwerveController;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private RobotContainer _robotContainer;
	private LoggedDashboardChooser<Command> _autoChooser;
	private Command _autoCommand;

	public Robot(){
		boolean replayLastGame = false;
		if (!(replayLastGame && isSimulation())) {
//			 Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
		} else {
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}
		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
	}

	@Override
	public void robotInit() {
		_robotContainer = new RobotContainer();
		// _autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
	}

	@Override
	public void robotPeriodic() {
    	Threads.setCurrentThreadPriority(true, 99);
		CommandScheduler.getInstance().run();
		_robotContainer.periodic();
		Threads.setCurrentThreadPriority(false, 10);
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		SwerveController.getInstance().setState(SwerveDemand.SwerveState.VELOCITY);
		SwerveController.getInstance().Demand.fieldRelative = false;
		RobotState.getInstance().setRobotState(RobotStates.CORAL_READY);
		_autoCommand = _autoChooser.get();

		if (_autoCommand != null)
			_autoCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (_autoCommand != null)
			_autoCommand.cancel();

		_robotContainer.resetSubsystems();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		if (_autoCommand != null)
			_autoCommand.cancel();

		CommandScheduler.getInstance().cancelAll();
		RobotState.getInstance().setRobotState(RobotStates.TEST);
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
