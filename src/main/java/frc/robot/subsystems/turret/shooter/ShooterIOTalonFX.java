package frc.robot.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO
{
    protected final TalonFX m_motor1;
    protected final TalonFX m_motor2;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final StatusSignal<AngularVelocity> m_velocity;
    protected final Supplier<Boolean> m_isPresent;
    protected final MotionMagicVelocityVoltage m_motionMagicVoltage;
    protected final AngularVelocity m_errorTolerance;

    private AngularVelocity m_targetSpeed = RotationsPerSecond.of(0);

    public ShooterIOTalonFX(ShooterIO.ShooterConstants constants)
    {
        this(constants.kMotor1ID(), constants.kMotor2ID(), constants.kS(), constants.kV(), constants.kA(),
                constants.kP(), constants.kI(), constants.kD(), constants.kG(), constants.kCruiseVelocity(),
                constants.kAcceleration(), constants.kJerk(), constants.kMotor1Inverted(), constants.kMotor2Inverted(),
                constants.kErrorTolerance());
    }

    public ShooterIOTalonFX(int kMotor1ID, int kMotor2ID, double kS, double kV, double kA, double kP, double kI,
            double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk, boolean kMotor1Inverted,
            boolean kMotor2Inverted, AngularVelocity kErrorTolerance)
    {
        m_motor1 = new TalonFX(kMotor1ID);
        m_motor2 = new TalonFX(kMotor2ID);
        TalonFXConfiguration config = new TalonFXConfiguration();

        var slot0Configs = config.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kG = kG;

        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = kCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
        motionMagicConfigs.MotionMagicJerk = kJerk;

        m_motor1.getConfigurator().apply(config);
        m_motor2.getConfigurator().apply(config);

        m_motor1.getConfigurator().apply(new MotorOutputConfigs().withInverted(
                kMotor1Inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive));
        m_motor2.getConfigurator().apply(new MotorOutputConfigs().withInverted(
                kMotor2Inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive));

        m_temperature = m_motor1.getDeviceTemp();
        m_voltage = m_motor1.getMotorVoltage();
        m_current = m_motor1.getTorqueCurrent();
        m_velocity = m_motor1.getVelocity();
        m_isPresent = () -> m_motor1.isConnected() && m_motor2.isConnected();

        m_motionMagicVoltage = new MotionMagicVelocityVoltage(0).withEnableFOC(true);
        m_errorTolerance = kErrorTolerance;
    }

    @Override
    public void update()
    {
        StatusSignal.refreshAll(m_temperature, m_voltage, m_current, m_velocity);
    }

    @Override
    public void setTargetSpeed(AngularVelocity speed)
    {
        m_targetSpeed = speed;
        m_motor1.setControl(m_motionMagicVoltage.withVelocity(speed));
        m_motor2.setControl(m_motionMagicVoltage.withVelocity(speed));
    }

    @Override
    public AngularVelocity getTargetSpeed()
    {
        return m_targetSpeed;
    }

    @Override
    public AngularVelocity getSpeed()
    {
        return m_velocity.getValue();
    }

    @Override
    public boolean isAtTargetSpeed()
    {
        return Math.abs(getTargetSpeed().in(RotationsPerSecond) - getSpeed().in(RotationsPerSecond)) < m_errorTolerance
                .in(RotationsPerSecond);
    }
}