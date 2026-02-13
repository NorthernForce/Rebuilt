package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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

    public HoodIOTalonFXS(HoodConstants constants)
    {
        this(constants.kMotorID(), constants.kEncoderID(), constants.kS(), constants.kV(), constants.kA(),
                constants.kP(), constants.kI(), constants.kD(), constants.kG(), constants.kCruiseVelocity(),
                constants.kAcceleration(), constants.kJerk(), constants.kGearRatio(), constants.kInverted(),
                constants.kLowerSoftLimit(), constants.kUpperSoftLimit(), constants.kErrorTolerance(),
                constants.kMotorArrangement());
    }

    private HoodIOTalonFXS(int kMotorID, int kEncoderID, double kS, double kV, double kA, double kP, double kI,
            double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk, double kGearRatio,
            boolean kInverted, Angle kLowerSoftLimit, Angle kUpperSoftLimit, Angle kErrorTolerance,
            MotorArrangementValue kMotorArrangement)
    {
        m_motor = new TalonFXS(kMotorID);
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
        motionMagicConfigs.MotionMagicCruiseVelocity = kCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
        motionMagicConfigs.MotionMagicJerk = kJerk;

        config.MotorOutput.Inverted = kInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.ExternalFeedback.RotorToSensorRatio = kGearRatio;
        config.ExternalFeedback.FeedbackRemoteSensorID = kEncoderID;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kUpperSoftLimit.in(Degrees);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kLowerSoftLimit.in(Degrees);

        config.Commutation.MotorArrangement = kMotorArrangement;

        m_motor.getConfigurator().apply(config);

        m_position = m_motor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getTorqueCurrent();
        m_velocity = m_motor.getVelocity();
        m_rotorVelocity = m_motor.getRotorVelocity();
        m_isPresent = () -> m_motor.isConnected() && !m_motor.getFault_HallSensorMissing().getValue();

        m_motionMagicVoltage = new MotionMagicExpoVoltage(0).withEnableFOC(true);
        m_errorTolerance = kErrorTolerance;
    }

    @Override
    public void update()
    {
        StatusSignal.refreshAll(m_position, m_temperature, m_voltage, m_current, m_velocity, m_rotorVelocity);
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        m_targetAngle = angle;
        m_motor.setControl(m_motionMagicVoltage.withPosition(angle.in(Rotations)));
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