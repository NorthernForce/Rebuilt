package frc.robot.lobby.subsystems.flicker;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class FlickerIOSparkMax implements FlickerIO
{
    private Spark m_motor;
    private int m_id;
    private double m_rampSpeed;

    public FlickerIOSparkMax(FlickerParameters parameters)
    {
        m_id = parameters.motorId();
        m_motor = new Spark(m_id);
        m_rampSpeed = parameters.rampSpeed();
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