package frc.robot.lobby.subsystems.spindexer.flicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class FlickerIOSparkMax implements FlickerIO
{
    private SparkMax m_motor;
    private int m_id;
    private double m_rampSpeed;
    private double m_errorTolerance;
    private double nanoTimeLastChecked = 0.0;
    private final Current jamCurrentThreshold;
    private final Time jamTimeout;
    private final double dejamSpeed;

    public FlickerIOSparkMax(FlickerParameters parameters)
    {
        m_id = parameters.motorId();
        m_motor = new SparkMax(m_id, MotorType.kBrushless);
        m_rampSpeed = parameters.rampSpeed();
        m_errorTolerance = parameters.errorTolerance();
        jamCurrentThreshold = parameters.jamCurrentThreshold();
        jamTimeout = parameters.jamTimeout();
        dejamSpeed = parameters.dejamSpeed();
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

    @Override
    public boolean getJammed()
    {
        double currentTime = System.nanoTime();
        if (m_motor.getOutputCurrent() > jamCurrentThreshold.in(Amps))
        {
            if (currentTime - nanoTimeLastChecked > jamTimeout.in(Seconds) * Math.pow(10, 9))
            {
                return true;
            } else
            {
                return false;
            }
        } else
        {
            nanoTimeLastChecked = currentTime;
            return false;
        }
    }

    @Override
    public void dejam()
    {
        m_motor.set(-dejamSpeed);
    }

    @Override
    public void resetJamDetection()
    {
        nanoTimeLastChecked = System.nanoTime();
    }
}