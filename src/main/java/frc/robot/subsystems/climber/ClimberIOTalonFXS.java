package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue; // Import for Limit Switch

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations; // Import Constants
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import frc.robot.lobby.LobbyConstants;

public class ClimberIOTalonFXS implements ClimberIO
{

    private final TalonFXS motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ClimberIOTalonFXS(int canId)
    {
        motor = new TalonFXS(canId);

        var config = new TalonFXSConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = LobbyConstants.ClimberConstants.kCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kG = LobbyConstants.ClimberConstants.kG;
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

        // Read Limit Switch
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
        // Convert Meters back to Rotations
        double rotations = positionMeters / LobbyConstants.ClimberConstants.kMetersPerRotation;
        motor.setPosition(rotations);
    }
}