package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ClimberIOTalonFXS implements ClimberIO
{

    private final TalonFXS motor;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Constructor: Needs the CAN ID of the motor
    public ClimberIOTalonFXS(int canId)
    {
        motor = new TalonFXS(canId);

        // Configure the motor
        var config = new TalonFXSConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // Hold position when stopped

        // Safety: Limit current to prevent burning out the motor
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        // Read data from the motor
        inputs.positionMeters = motor.getPosition().getValue().in(Rotations); // TODO: Convert Rotations to Meters
        inputs.velocityMetersPerSec = motor.getVelocity().getValue().in(RotationsPerSecond); // TODO: Convert
        inputs.appliedVolts = motor.getMotorVoltage().getValue().in(Volts);
        inputs.currentAmps = motor.getStatorCurrent().getValue().in(Amps);
        inputs.tempCelsius = motor.getDeviceTemp().getValue().in(Celsius);
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
}