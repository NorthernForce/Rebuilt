package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.lobby.LobbyConstants;

public class ClimberIOTalonFX implements ClimberIO
{

    private final TalonFX motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ClimberIOTalonFX(int canId)
    {
        motor = new TalonFX(canId);

        var config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;

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
        inputs.atBottomLimit = motor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
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
}