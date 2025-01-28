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
        return _instance;
    }

    public static void createInstance(boolean paused){
        _instance = new Leds(paused);
    }

    AddressableLED _leds;
    AddressableLEDBuffer _ledsBuffer;
    Timer _timer;

    private Leds(boolean paused) {
        super(paused);

        if(!_paused) {
            _leds = new AddressableLED(LedsConstants.kLedsPort);
            _ledsBuffer = new AddressableLEDBuffer(LedsConstants.kBuffer);
            _leds.setLength(_ledsBuffer.getLength());
            _leds.start();
        }
    }

    public void setPattern(LEDPattern _pattern) {
        if(!_paused) {
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
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kBlack)), RobotStates.RESET, RobotStates.CORAL_SEARCH);
        addFunctionToOnChangeMap(this::intakeLights, RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kWhite)), RobotStates.CORAL_READY, RobotStates.OUTTAKE_READY);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(1.5))), RobotStates.TEST);
        addFunctionToOnChangeMap(this::rainbow, RobotStates.REMOVE_ALGAE);
    }
}
