package frc.robot.lobby.subsystems.spindexer.flicker;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class FlickerIOSparkMax implements FlickerIO
{
    private Spark m_motor;
    private int m_id;
    private double m_rampSpeed;
    private double m_errorTolerance;

    public FlickerIOSparkMax(FlickerParameters parameters)
    {
        m_id = parameters.motorId();
        m_motor = new Spark(m_id);
        m_rampSpeed = parameters.rampSpeed();
        m_errorTolerance = parameters.errorTolerance();
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

    @Override
    public double getPower()
    {
        return m_motor.get();
    }

    @Override
    public void setPower(double power)
    {
        m_rampSpeed = power;
    }

    @Override
    public double getTargetPower()
    {
        return m_rampSpeed;
    }
}