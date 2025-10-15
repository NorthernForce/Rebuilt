package frc.robot.subsystems.superstructure.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhoenixUtil;

public class Elevator extends SubsystemBase
{
    public final TalonFX motor;
    public final ElevatorConfig config;
    public final MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
    public final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    public final StatusSignal<Angle> position;

    private Distance goalPosition;

    public Elevator(int canID, int sensorID, ElevatorConfig config)
    {
        motor = new TalonFX(canID);
        configureMotor(config);
        this.config = config;
        position = motor.getPosition();
    }

    private void configureMotor(ElevatorConfig constants)
    {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kS = constants.kS;
        config.Slot0.kV = constants.kV;
        config.Slot0.kA = constants.kA;
        config.Slot0.kP = constants.kP;
        config.Slot0.kI = constants.kI;
        config.Slot0.kD = constants.kD;
        config.Slot0.kG = constants.kG;
        config.MotionMagic.MotionMagicCruiseVelocity = constants.kCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = constants.kAcceleration;
        config.MotionMagic.MotionMagicJerk = constants.kJerk;
        config.MotorOutput.Inverted = constants.kInverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.SensorToMechanismRatio = constants.kGearRatio / constants.kSprocketCircumference.in(Inches);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.kUpperLimit.in(Inches);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.kLowerLimit.in(Inches);
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    public void stop()
    {
        motor.setControl(dutyCycleOut.withOutput(0 + config.kG / 12.0));
    }

    public void manualControl(double speed)
    {
        motor.setControl(dutyCycleOut.withOutput(speed));
    }

    public void setGoalPosition(Distance position)
    {
        this.goalPosition = position;
        motor.setControl(motionMagicExpoVoltage.withPosition(position.in(Meters)));
    }

    public boolean isAtPosition(Distance position)
    {
        return Math.abs(getPosition().in(Inches) - goalPosition.in(Inches)) < config.kErrorTolerance.in(Inches);
    }

    public boolean isAtGoal()
    {
        return isAtPosition(goalPosition);
    }

    public void update()
    {
        BaseStatusSignal.refreshAll(position);
    }

    public Distance getPosition()
    {
        return Inches.of(position.getValue().in(Degrees));
    }

    public record ElevatorConfig(double kS, double kV, double kA, double kP, double kI, double kD, double kG,
            double kCruiseVelocity, double kAcceleration, double kJerk, Distance kSprocketCircumference,
            double kGearRatio, boolean kInverted, Distance kLowerLimit, Distance kUpperLimit, Mass kMass,
            Distance kErrorTolerance) {

    }
}
