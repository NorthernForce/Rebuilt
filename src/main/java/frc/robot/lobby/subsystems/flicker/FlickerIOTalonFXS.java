package frc.robot.lobby.subsystems.flicker;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlickerIOTalonFXS implements FlickerIO
{
    private int m_motorId;
    private TalonFXS m_motor;
    private double m_rampSpeed;

    public FlickerIOTalonFXS(FlickerParameters parameters)
    {
        m_motorId = parameters.motorId();
        m_rampSpeed = parameters.rampSpeed();
        m_motor = new TalonFXS(m_motorId);
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void rampFlicker()
    {
        m_motor.set(m_rampSpeed);
    }

    @Override
    public void stopFlicker()
    {
        m_motor.set(0.0);
    }
}
