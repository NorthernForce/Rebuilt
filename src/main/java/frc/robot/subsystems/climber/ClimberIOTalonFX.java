package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lobby.LobbyConstants;

public class ClimberIOTalonFX implements ClimberIO
{

    private final TalonFX motor;
    private final DigitalInput limitSwitch;
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ClimberIOTalonFX(int canId)
    {
        motor = new TalonFX(canId);
        limitSwitch = new DigitalInput(LobbyConstants.ClimberConstants.kLimitSwitchId);

        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = LobbyConstants.ClimberConstants.kCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kP = LobbyConstants.ClimberConstants.kP;
        config.Slot0.kI = LobbyConstants.ClimberConstants.kI;
        config.Slot0.kD = LobbyConstants.ClimberConstants.kD;
        config.Slot0.kG = LobbyConstants.ClimberConstants.kG;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        inputs.positionMeters = motor.getPosition().getValue().in(Rotations)
                * LobbyConstants.ClimberConstants.kMetersPerRotation;
        inputs.velocityMetersPerSec = motor.getVelocity().getValue().in(RotationsPerSecond)
                * LobbyConstants.ClimberConstants.kMetersPerRotation;
        inputs.appliedVolts = motor.getMotorVoltage().getValue().in(Volts);
        inputs.currentAmps = motor.getStatorCurrent().getValue().in(Amps);
        inputs.tempCelsius = motor.getDeviceTemp().getValue().in(Celsius);

        inputs.atBottomLimit = !limitSwitch.get();
    }

    @Override
    public void setVoltage(double volts)
    {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop()
    {
        motor.setControl(voltageRequest.withOutput(0));
    }

    @Override
    public void setPosition(double positionMeters)
    {
        double rotations = positionMeters / LobbyConstants.ClimberConstants.kMetersPerRotation;
        motor.setPosition(rotations);
    }

    @Override
    public void setPositionControl(double positionMeters)
    {
        double rotations = positionMeters / LobbyConstants.ClimberConstants.kMetersPerRotation;
        motor.setControl(positionRequest.withPosition(rotations));
    }
}