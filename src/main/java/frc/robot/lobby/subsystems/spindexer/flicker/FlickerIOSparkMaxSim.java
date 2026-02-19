package frc.robot.lobby.subsystems.spindexer.flicker;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;

public class FlickerIOSparkMaxSim implements FlickerIO
{
    private int m_id;
    private double m_rampSpeed;
    private double m_errorTolerance;
    private double m_simMaxRpm;
    private SparkMaxSim m_motor;

    public FlickerIOSparkMaxSim(FlickerSimParameters parameters)
    {
        m_id = parameters.motorId();
        m_rampSpeed = parameters.rampSpeed();
        m_errorTolerance = parameters.errorTolerance();
        m_simMaxRpm = parameters.simMaxRpm();
        DCMotor motorType = DCMotor.getNEO(1);
        SparkMax sparkMax = new SparkMax(m_id, SparkMax.MotorType.kBrushless);
        m_motor = new SparkMaxSim(sparkMax, motorType);
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
}
