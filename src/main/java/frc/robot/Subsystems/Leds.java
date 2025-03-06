package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.LedsConstants;
import frc.robot.StateMachine.RobotStates;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.LedsConstants.kLedSpacing;

public class Leds extends StateMachineSubsystem<RobotStates> {
    private static Leds _instance;

    public static Leds getInstance(){
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Leds(paused);
    }

    AddressableLED _leds;
    AddressableLEDBuffer _ledsBuffer;
    AddressableLEDBufferView _leftBuffer;
    AddressableLEDBufferView _rightBuffer;
    Timer _timer = new Timer();

    private Leds(boolean paused) {
        super(paused);

        if(_paused)
            return;

        _leds = new AddressableLED(LedsConstants.kPort);
        _ledsBuffer = new AddressableLEDBuffer(LedsConstants.kLength);
        _leds.setLength(_ledsBuffer.getLength());
        _leftBuffer = new AddressableLEDBufferView(_ledsBuffer, 0, 16);
        _rightBuffer = new AddressableLEDBufferView(_ledsBuffer, 17, 32);
        _leds.setData(_ledsBuffer);
        _leds.start();

        double time = 4;
        Commands.sequence(
            Commands.runOnce(() -> _timer.get()),
            Commands.run(() -> setPattern(LEDPattern.solid(Color.kPurple).mask(LEDPattern.progressMaskLayer(() -> _timer.get() / time)))).until(() -> _timer.get() >= time)
        ).schedule();
    }

    public void setPattern(LEDPattern _pattern) {
        if(!_paused){
            _pattern.applyTo(_leftBuffer);
            _pattern.applyTo(_rightBuffer);
        }
    }

    public void intakeLights() {
        if(!_paused) {
            _timer.restart();
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> _timer.get());
            setPattern(_pattern);
        }
    }

    public void outtakeLights() {
        if(!_paused) {
            _timer.restart();
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> 1 - _timer.get());
            setPattern(_pattern);
        }
    }

    public void elevatorUp(double current, double end) {
        if(!_paused) {
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> current / end);
            setPattern(_pattern);
        }
    }

    public void elevatorDown(double current, double end) {
        if(!_paused) {
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> 1 - current / end);
            setPattern(_pattern);
        }
    }

    public void rainbow() {
        if(!_paused) {
            LEDPattern _pattern = LEDPattern.rainbow(255, 128);
            LEDPattern _scrollingRainbow = _pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
            setPattern(_scrollingRainbow);
        }
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kWhite)), RobotStates.CORAL_SEARCH);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kDarkRed).blink(Seconds.of(0.125))), RobotStates.RESET);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.125))), RobotStates.CLOSE);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.25))), RobotStates.INTAKE, RobotStates.INDEX_BACK, RobotStates.INDEX);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kGreen)), RobotStates.CORAL_READY, RobotStates.OUTTAKE_READY);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.25))), RobotStates.TEST);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.25))), RobotStates.GO_RIGHT_REEF, RobotStates.GO_LEFT_REEF);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kPurple)), RobotStates.AT_SIDE_REEF);
        addFunctionToPeriodicMap(this::rainbow, RobotStates.REMOVE_ALGAE);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(!_paused){
            _leds.setData(_ledsBuffer);
        }
    }
}
