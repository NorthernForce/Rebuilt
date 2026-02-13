package frc.robot.lobby.subsystems.flicker;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlickerIOTalonFXS implements FlickerIO
{
    private TalonFXS m_motor;
    private double m_rampSpeed;
    private double m_gearRatio;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public FlickerIOTalonFXS(FlickerParameters parameters)
    {
        m_rampSpeed = parameters.rampSpeed();
        m_gearRatio = parameters.gearRatio();
        m_motor = new TalonFXS(parameters.motorId());

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kV = parameters.kV();
        config.Slot0.kP = parameters.kP();
        config.Slot0.kI = parameters.kI();
        config.Slot0.kD = parameters.kD();
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void rampFlicker()
    {
        // Target is mechanism RPS, but TalonFXS expects rotor RPS
        double targetRotorRps = m_rampSpeed * m_gearRatio;
        m_motor.setControl(m_velocityRequest.withVelocity(targetRotorRps));
    }

    @Override
    public void stopFlicker()
    {
        m_motor.setControl(m_velocityRequest.withVelocity(0));
    }

    @Override
    public double getSpeed()
    {
        // Return mechanism RPS
        return m_motor.getVelocity().getValueAsDouble() / m_gearRatio;
    }
}
