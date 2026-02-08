package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.turret.suzie.SuzieIO;

public class HoodIOTalonFXS implements HoodIO
{
    protected final TalonFXS m_motor;
    protected final StatusSignal<Angle> m_position;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final StatusSignal<AngularVelocity> m_velocity;
    protected final StatusSignal<AngularVelocity> m_rotorVelocity;
    protected final Supplier<Boolean> m_isPresent;
    protected final MotionMagicExpoVoltage m_motionMagicVoltage;
    protected final Angle m_errorTolerance;

    private Angle m_targetAngle = Degrees.of(0);

    public HoodIOTalonFXS(SuzieIO.SuzieConstants constants)
    {
        this(constants.motorID(), constants.encoderID(), constants.kS(), constants.kV(), constants.kA(), constants.kP(),
                constants.kI(), constants.kD(), constants.kG(), constants.cruiseVelocity(), constants.acceleration(),
                constants.jerk(), constants.gearRatio(), constants.inverted(), constants.lowerSoftLimit(),
                constants.upperSoftLimit(), constants.errorTolerance(), constants.motorArrangement());
    }

    private HoodIOTalonFXS(int motorID, int encoderID, double kS, double kV, double kA, double kP, double kI, double kD,
            double kG, double cruiseVelocity, double acceleration, double jerk, double gearRatio, boolean inverted,
            Angle lowerSoftLimit, Angle upperSoftLimit, Angle errorTolerance, MotorArrangementValue motorArrangement)
    {
        m_motor = new TalonFXS(motorID);
        TalonFXSConfiguration config = new TalonFXSConfiguration();

        var slot0Configs = config.Slot0;
        slot0Configs.kS = kS;
        slot0Configs.kV = kV;
        slot0Configs.kA = kA;
        slot0Configs.kP = kP;
        slot0Configs.kI = kI;
        slot0Configs.kD = kD;
        slot0Configs.kG = kG;

        var motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = acceleration;
        motionMagicConfigs.MotionMagicJerk = jerk;

        config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.ExternalFeedback.RotorToSensorRatio = gearRatio;
        config.ExternalFeedback.FeedbackRemoteSensorID = encoderID;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = upperSoftLimit.in(Degrees);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = lowerSoftLimit.in(Degrees);

        config.Commutation.MotorArrangement = motorArrangement;

        m_motor.getConfigurator().apply(config);

        m_position = m_motor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getTorqueCurrent();
        m_velocity = m_motor.getVelocity();
        m_rotorVelocity = m_motor.getRotorVelocity();
        m_isPresent = () -> m_motor.isConnected() && !m_motor.getFault_HallSensorMissing().getValue();

        m_motionMagicVoltage = new MotionMagicExpoVoltage(0).withEnableFOC(true);
        m_errorTolerance = errorTolerance;
    }

    @Override
    public void updateStatusSignals()
    {
        StatusSignal.refreshAll(m_position, m_temperature, m_voltage, m_current, m_velocity, m_rotorVelocity);
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        m_targetAngle = angle;
        m_motor.setControl(m_motionMagicVoltage.withPosition(angle.in(Degrees)));
    }

    @Override
    public Angle getTargetAngle()
    {
        return m_targetAngle;
    }

    @Override
    public Angle getAngle()
    {
        return m_position.getValue();
    }

    @Override
    public boolean isAtTargetAngle()
    {
        return Math.abs(getTargetAngle().in(Degrees) - getAngle().in(Degrees)) < m_errorTolerance.in(Degrees);
    }
}