package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.RelativeEncoder;

public class ClimberIOSparkMax implements ClimberIO
{

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    public ClimberIOSparkMax(int canId)
    {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        // Configure the Spark Max (2025/2026 REV API style)
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake); // Hold position
        config.smartCurrentLimit(40); // Safety limit

        // Apply config
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        inputs.positionMeters = encoder.getPosition(); // TODO: Convert Rotations to Meters
        inputs.velocityMetersPerSec = encoder.getVelocity(); // TODO: Convert RPM to Meters/Sec
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(double volts)
    {
        motor.setVoltage(volts);
    }

    @Override
    public void stop()
    {
        motor.setVoltage(0);
    }
}