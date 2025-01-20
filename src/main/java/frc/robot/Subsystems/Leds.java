package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LedsConstants;
import frc.robot.StateMachine.RobotStates;
import frc.robot.StateMachine.StateMachine;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LedsConstants.kLedSpacing;

public class Leds extends StateMachineSubsystem<RobotStates> {
    private static Leds _instance;

    public static Leds getInstance(){
        if(_instance == null)
            _instance = new Leds();
        return _instance;
    }

    AddressableLED _leds;
    AddressableLEDBuffer _ledsBuffer;
    Timer _timer;

    private Leds() {
        _leds = new AddressableLED(LedsConstants.kLedsPort);
        _ledsBuffer = new AddressableLEDBuffer(LedsConstants.kBuffer);
        _leds.setLength(_ledsBuffer.getLength());
        _leds.start();
    }


    public void setPattern(LEDPattern _pattern) {
        _pattern.applyTo(_ledsBuffer);
        _leds.setData(_ledsBuffer);
    }

    public void intakeLights() {
        _timer.restart();
        LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> _timer.get());
        setPattern(_pattern);
    }

    public void outtakeLights() {
        _timer.restart();
        LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> 1 - _timer.get());
        setPattern(_pattern);
    }

    public void elevatorUp(double current, double end) {
        LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> end/current);
        setPattern(_pattern);
    }

    public void elevatorDown(double current, double end) {
        LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> 1 - end/current);
        setPattern(_pattern);
    }

    public void rainbow() {
        LEDPattern _pattern = LEDPattern.rainbow(255, 128);
        LEDPattern _scrollingRainbow = _pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
        setPattern(_scrollingRainbow);
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kBlack)), RobotStates.RESET, RobotStates.CORAL_SEARCH, RobotStates.INTAKE_PREPARE);
        addFunctionToOnChangeMap(this::intakeLights, RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kWhite)), RobotStates.CORAL_READY, RobotStates.OUTTAKE_READY);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(1.5))), RobotStates.TEST);
        addFunctionToOnChangeMap(this::rainbow, RobotStates.REMOVE_ALGAE);
        addFunctionToOnChangeMap(() -> elevatorUp(Elevator.getInstance().getPosition(), Elevator.getInstance().getGoal()), RobotStates.REMOVE_ALGAE);
        addFunctionToOnChangeMap(() -> elevatorDown(Elevator.getInstance().getPosition(), Elevator.getInstance().getGoal()), RobotStates.REMOVE_ALGAE);




    }
}