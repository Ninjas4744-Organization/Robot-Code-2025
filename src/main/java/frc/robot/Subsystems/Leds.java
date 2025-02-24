package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
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

        if(!_paused) {
            _leds = new AddressableLED(LedsConstants.kPort);
            _ledsBuffer = new AddressableLEDBuffer(LedsConstants.kLength);
//            _leftBuffer = _ledsBuffer.createView(0, 19);
//            _rightBuffer = _ledsBuffer.createView(20, 38);
            _leds.setLength(_ledsBuffer.getLength());
            LEDPattern.solid(Color.kRed).applyTo(_ledsBuffer);
            _leds.setData(_ledsBuffer);
            _leds.start();
        }
    }

    public void setPattern(LEDPattern _pattern) {
        if(!_paused){
//            _pattern.applyTo(_leftBuffer);
//            _pattern.applyTo(_rightBuffer);
            _pattern.applyTo(_ledsBuffer);
            _leds.setData(_ledsBuffer);
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

    public void rainbow() {
        if(!_paused) {
            LEDPattern _pattern = LEDPattern.rainbow(255, 128);
            LEDPattern _scrollingRainbow = _pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
            setPattern(_scrollingRainbow);
        }
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kRed)), RobotStates.CORAL_SEARCH);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kDarkRed).blink(Seconds.of(0.5))), RobotStates.RESET);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5))), RobotStates.CLOSE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreenYellow).blink(Seconds.of(0.5))), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreenYellow).blink(Seconds.of(0.25))), RobotStates.INDEX_BACK);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreenYellow).blink(Seconds.of(0.125))), RobotStates.INDEX);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreen)), RobotStates.CORAL_READY, RobotStates.OUTTAKE_READY);
        addFunctionToPeriodicMap(() -> setPattern(LEDPattern.solid(Color.kRed)), RobotStates.TEST);
        addFunctionToOnChangeMap(this::rainbow, RobotStates.REMOVE_ALGAE);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(!_paused){
            _leds.setData(_ledsBuffer);
        }
    }
}
