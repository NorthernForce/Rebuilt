package frc.robot.lobby.subsystems.flicker;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;

public class FlickerIOTalonFXSSim implements FlickerIO
{
    private int m_motorId;
    private double m_rampSpeed;
    private TalonFXSSimState m_motor;
    private double m_simMaxRpm;
    private TalonFXS m_motorController;

    public FlickerIOTalonFXSSim(FlickerSimParameters parameters)
    {
        m_motorId = parameters.motorId();
        m_rampSpeed = parameters.rampSpeed();
        m_motorController = new TalonFXS(m_motorId);
        m_motor = m_motorController.getSimState();
        m_simMaxRpm = parameters.simMaxRpm();
    }

    @Override
    public void rampFlicker()
    {
        m_motor.setRotorVelocity(m_rampSpeed * (m_simMaxRpm / 60.0));
    }

    @Override
    public void stopFlicker()
    {
        m_motor.setRotorVelocity(0.0);
    }
}
