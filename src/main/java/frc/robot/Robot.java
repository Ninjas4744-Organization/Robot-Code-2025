package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.StateMachine.RobotState;
import frc.robot.StateMachine.RobotStates;

public class Robot extends TimedRobot {
	public static Robot instance;
	private RobotContainer _robotContainer;
	private SendableChooser<String> _autoChooser;
	private Command _autoCommand;

	@Override
	public void robotInit() {
		_autoChooser = new SendableChooser<>();
		_autoChooser.setDefaultOption("None", "None");
		_autoChooser.addOption("", "");
		SmartDashboard.putData("Autonomy", _autoChooser);

		instance = this;
		_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		_robotContainer.periodic();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		_autoCommand = CommandBuilder.Auto.autoCommand(_autoChooser.getSelected());

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
		CommandScheduler.getInstance().cancelAll();
		RobotState.getInstance().setRobotState(RobotStates.TESTING);
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
