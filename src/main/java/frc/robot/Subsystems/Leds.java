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
    private static boolean _dontCreate = false;

    public static Leds getInstance(){
        if(_instance == null)
            _instance = new Leds();
        return _instance;
    }

    AddressableLED _leds;
    AddressableLEDBuffer _ledsBuffer;
    Timer _timer;

    private Leds() {
        super();
        if(!_dontCreate) {
            _leds = new AddressableLED(LedsConstants.kLedsPort);
            _ledsBuffer = new AddressableLEDBuffer(LedsConstants.kBuffer);
            _leds.setLength(_ledsBuffer.getLength());
            _leds.start();
        }
    }

    public void setPattern(LEDPattern _pattern) {
        if(!_dontCreate) {
            _pattern.applyTo(_ledsBuffer);
            _leds.setData(_ledsBuffer);
        }
    }

    public void intakeLights() {
        if(!_dontCreate) {
            _timer.restart();
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> _timer.get());
            setPattern(_pattern);
        }
    }

    public void outtakeLights() {
        if(!_dontCreate) {
            _timer.restart();
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> 1 - _timer.get());
            setPattern(_pattern);
        }
    }

    public void elevatorUp(double current, double end) {
        if(!_dontCreate) {
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> end / current);
            setPattern(_pattern);
        }
    }

    public void elevatorDown(double current, double end) {
        if(!_dontCreate) {
            LEDPattern _pattern = LEDPattern.progressMaskLayer(() -> 1 - end / current);
            setPattern(_pattern);
        }
    }

    public void rainbow() {
        if(!_dontCreate) {
            LEDPattern _pattern = LEDPattern.rainbow(255, 128);
            LEDPattern _scrollingRainbow = _pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
            setPattern(_scrollingRainbow);
        }
    }

    public static void dontCreateSubsystem(){
        _dontCreate = true;
    }

    @Override
    protected void setFunctionMaps() {
        if(_dontCreate)
            return;
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kBlack)), RobotStates.RESET, RobotStates.CORAL_SEARCH);
        addFunctionToOnChangeMap(this::intakeLights, RobotStates.INTAKE);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kWhite)), RobotStates.CORAL_READY, RobotStates.OUTTAKE_READY);
        addFunctionToOnChangeMap(() -> setPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(1.5))), RobotStates.TEST);
        addFunctionToOnChangeMap(this::rainbow, RobotStates.REMOVE_ALGAE);
        addFunctionToOnChangeMap(() -> elevatorUp(Elevator.getInstance().getPosition(), Elevator.getInstance().getGoal()), RobotStates.REMOVE_ALGAE);
        addFunctionToOnChangeMap(() -> elevatorDown(Elevator.getInstance().getPosition(), Elevator.getInstance().getGoal()), RobotStates.REMOVE_ALGAE);
    }
}