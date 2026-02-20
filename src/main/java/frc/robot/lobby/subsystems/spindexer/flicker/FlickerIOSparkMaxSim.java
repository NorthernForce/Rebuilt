package frc.robot.lobby.subsystems.spindexer.flicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class FlickerIOSparkMaxSim implements FlickerIO
{
    private int m_id;
    private double m_rampSpeed;
    private double m_errorTolerance;
    private double m_simMaxRpm;
    private SparkMaxSim m_motor;
    private double nanoTimeLastChecked = 0.0;
    private final Current jamCurrentThreshold;
    private final Time jamTimeout;
    private final double dejamSpeed;

    public FlickerIOSparkMaxSim(FlickerSimParameters parameters)
    {
        m_id = parameters.motorId();
        m_rampSpeed = parameters.rampSpeed();
        m_errorTolerance = parameters.errorTolerance();
        m_simMaxRpm = parameters.simMaxRpm();
        DCMotor motorType = DCMotor.getNEO(1);
        SparkMax sparkMax = new SparkMax(m_id, SparkMax.MotorType.kBrushless);
        m_motor = new SparkMaxSim(sparkMax, motorType);
        jamCurrentThreshold = parameters.jamCurrentThreshold();
        jamTimeout = parameters.jamTimeout();
        dejamSpeed = parameters.dejamSpeed();

    }

    @Override
    public void rampFlicker()
    {
        m_motor.setVelocity(m_rampSpeed * (m_simMaxRpm / 60.0));
    }

    @Override
    public void stopFlicker()
    {
        m_motor.setVelocity(0.0);
    }

    @Override
    public double getPower()
    {
        return m_rampSpeed;
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
        if (m_motor.getMotorCurrent() > jamCurrentThreshold.in(Amps))
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
        m_motor.setAppliedOutput(-dejamSpeed);
    }

    @Override
    public void resetJamDetection()
    {
        nanoTimeLastChecked = System.nanoTime();
    }
}
