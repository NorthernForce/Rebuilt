package frc.robot.lobby.subsystems.spindexer.carousel;

import static edu.wpi.first.units.Units.Amps;

import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class CarouselIOTalonFX implements CarouselIO
{
    protected final TalonFX m_motor;
    protected double m_speed;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final StatusSignal<AngularVelocity> m_velocity;
    protected final StatusSignal<AngularVelocity> m_rotorVelocity;
    protected final Supplier<Boolean> m_isPresent;
    protected final VelocityVoltage m_velocityVoltage;
    private final Current jamCurrentThreshold;
    private final Time jamTimeout;
    private double nanoTimeLastChecked = 0.0;
    private final double dejamSpeed;

    public CarouselIOTalonFX(CarouselConstants constants)
    {
        this(constants.kMotorId(), constants.kSpeed(), constants.kGearRatio(), constants.kInverted(),
                constants.kJamCurrentThreshold(), constants.kJamTimeout(), constants.dejamSpeed());
    }

    public CarouselIOTalonFX(int kMotorID, double kSpeed, double kGearRatio, boolean kInverted,
            Current kJamCurrentThreshold, Time kJamTimeout, double kDejamSpeed)
    {
        m_motor = new TalonFX(kMotorID);
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = kInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = kGearRatio;

        m_motor.getConfigurator().apply(config);

        m_speed = kSpeed;

        m_temperature = m_motor.getDeviceTemp();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getTorqueCurrent();
        m_velocity = m_motor.getVelocity();
        m_rotorVelocity = m_motor.getRotorVelocity();
        m_isPresent = () -> m_motor.isConnected();
        jamCurrentThreshold = kJamCurrentThreshold;
        jamTimeout = kJamTimeout;

        m_velocityVoltage = new VelocityVoltage(0);
        dejamSpeed = kDejamSpeed;

    }

    @Override
    public void startCarousel()
    {
        m_motor.set(m_speed);
    }

    @Override
    public void stopCarousel()
    {
        m_motor.set(0);
    }

    @Override
    public AngularVelocity getSpeed()
    {
        return m_velocity.getValue();
    }

    @Override
    public double getPower()
    {
        return m_motor.get();
    }

    @Override
    public void update()
    {
        StatusSignal.refreshAll(m_temperature, m_voltage, m_current, m_velocity, m_rotorVelocity);
    }

    @Override
    public void setPower(double power)
    {
        m_speed = power;
    }

    @Override
    public double getTargetPower()
    {
        return m_speed;
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
