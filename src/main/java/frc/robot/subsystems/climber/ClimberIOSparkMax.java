package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig; // Import for Limits
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.lobby.LobbyConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

public class ClimberIOSparkMax implements ClimberIO
{

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch reverseLimit;

    public ClimberIOSparkMax(int canId)
    {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        reverseLimit = motor.getReverseLimitSwitch();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        // Configure Limit Switch to be enabled
        config.limitSwitch.reverseLimitSwitchEnabled(true);
        config.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        inputs.positionMeters = encoder.getPosition() * LobbyConstants.ClimberConstants.kMetersPerRotation;
        inputs.velocityMetersPerSec = (encoder.getVelocity() / 60.0)
                * LobbyConstants.ClimberConstants.kMetersPerRotation; // RPM -> RPS -> MPS

        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();

        // Read Limit Switch
        inputs.atBottomLimit = reverseLimit.isPressed();
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

    @Override
    public void setPosition(double positionMeters)
    {
        double rotations = positionMeters / LobbyConstants.ClimberConstants.kMetersPerRotation;
        encoder.setPosition(rotations);
    }
}