package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lobby.LobbyConstants;
import com.revrobotics.RelativeEncoder;

public class ClimberIOSparkMax implements ClimberIO
{

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;

    public ClimberIOSparkMax(int canId)
    {
        motor = new SparkMax(canId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        limitSwitch = new DigitalInput(LobbyConstants.ClimberConstants.kLimitSwitchId);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        config.closedLoop.pid(LobbyConstants.ClimberConstants.kP, LobbyConstants.ClimberConstants.kI,
                LobbyConstants.ClimberConstants.kD);

        config.closedLoop.positionWrappingEnabled(false);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs)
    {
        inputs.positionMeters = encoder.getPosition() * LobbyConstants.ClimberConstants.kMetersPerRotation;
        inputs.velocityMetersPerSec = (encoder.getVelocity() / 60.0)
                * LobbyConstants.ClimberConstants.kMetersPerRotation;
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.tempCelsius = motor.getMotorTemperature();

        inputs.atBottomLimit = !limitSwitch.get();
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

    @Override
    public void setPositionControl(double positionMeters)
    {
        double rotations = positionMeters / LobbyConstants.ClimberConstants.kMetersPerRotation;
        motor.getClosedLoopController().setReference(rotations, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                LobbyConstants.ClimberConstants.kG);
    }
}