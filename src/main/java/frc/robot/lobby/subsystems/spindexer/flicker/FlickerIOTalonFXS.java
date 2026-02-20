package frc.robot.lobby.subsystems.spindexer.flicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public class FlickerIOTalonFXS implements FlickerIO
{
    private TalonFXS m_motor;
    private double m_rampSpeed;
    private double m_errorTolerance;
    private double m_gearRatio;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private double nanoTimeLastChecked = 0.0;
    private final Current jamCurrentThreshold;
    private final Time jamTimeout;
    private final double dejamSpeed;

    public FlickerIOTalonFXS(FlickerParameters parameters)
    {
        m_rampSpeed = parameters.rampSpeed();
        m_gearRatio = parameters.gearRatio();
        m_errorTolerance = parameters.errorTolerance();
        m_motor = new TalonFXS(parameters.motorId());
        jamCurrentThreshold = parameters.jamCurrentThreshold();
        jamTimeout = parameters.jamTimeout();
        m_motor.getConfigurator()
                .apply(new CommutationConfigs().withMotorArrangement(MotorArrangementValue.Minion_JST));
        m_motor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));
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
        m_motor.set(0);
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
        if (m_motor.getTorqueCurrent().getValueAsDouble() > jamCurrentThreshold.in(Amps))
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
