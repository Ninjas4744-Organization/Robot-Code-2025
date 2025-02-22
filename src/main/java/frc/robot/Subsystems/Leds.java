package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Subsystems.StateMachineSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
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

    AddressableLED _ledsLeft;
    AddressableLED _ledsRight;
    AddressableLEDBuffer _ledsBuffer;
    Timer _timer = new Timer();

    private Leds(boolean paused) {
        super(paused);

        if(!_paused) {
            _ledsLeft = new AddressableLED(LedsConstants.kLedsLeftPort);
//            _ledsRight = new AddressableLED(LedsConstants.kLedsRightPort);
            _ledsBuffer = new AddressableLEDBuffer(LedsConstants.kBuffer);
            _ledsLeft.setLength(_ledsBuffer.getLength());
            _ledsLeft.setData(_ledsBuffer);
            _ledsLeft.start();
        }
    }

    public void setPattern(LEDPattern _pattern) {
        if(!_paused)
            _pattern.applyTo(_ledsBuffer);
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
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kRed)), RobotStates.CORAL_SEARCH);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kDarkRed).blink(Seconds.of(0.5))), RobotStates.RESET);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5))), RobotStates.CLOSE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreenYellow).blink(Seconds.of(0.5))), RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreenYellow).blink(Seconds.of(0.25))), RobotStates.INDEX_BACK);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreenYellow).blink(Seconds.of(0.125))), RobotStates.INDEX);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kGreen)), RobotStates.CORAL_READY, RobotStates.OUTTAKE_READY);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5))), RobotStates.TEST);
        addFunctionToOnChangeMap(this::rainbow, RobotStates.REMOVE_ALGAE);
    }

    @Override
    public void periodic() {
        super.periodic();

        if(!_paused){
            _ledsLeft.setData(_ledsBuffer);
//            _ledsRight.setData(_ledsBuffer);
        }
    }
}
