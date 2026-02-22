package frc.robot.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.CRTEncodersUtil;

public class SuzieIOTalonFXS implements SuzieIO
{
    protected SuzieConstants constants;
    protected final TalonFXS m_motor;
    protected final DutyCycleEncoder m_drivingEncoder;
    protected final DutyCycleEncoder m_sensingEncoder;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final Supplier<Boolean> m_isPresent;
    protected final PositionVoltage m_positionVoltage;
    protected final Angle m_errorTolerance;

    private Angle m_targetAngle = Degrees.of(0);

    public SuzieIOTalonFXS(SuzieIO.SuzieConstants constants)
    {
        this(constants.kMotorID(), constants.kDrivingEncoderID(), constants.kSensingEncoderID(), constants.kS(), constants.kV(), constants.kA(),
            constants.kP(), constants.kI(), constants.kD(), constants.kG(), constants.kCruiseVelocity(),
            constants.kAcceleration(), constants.kJerk(), constants.kDrivingGearTeeth(), constants.kSensingGearTeeth(), constants.kTurntableGearTeeth(), constants.kInverted(),
            constants.kLowerSoftLimit(), constants.kUpperSoftLimit(), constants.kErrorTolerance(),
            constants.kMotorArrangement());
        this.constants = constants;
    }

    private SuzieIOTalonFXS(int kMotorID, int kDrivingEncoderID, int kSensingEncoderID, double kS, double kV, double kA, double kP,
            double kI, double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk,
            int kDrivingGearTeeth, int kSensingGearTeeth, int kTurntableGearTeeth, boolean kInverted, Angle kLowerSoftLimit, Angle kUpperSoftLimit, Angle kErrorTolerance,
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

        // var motionMagicConfigs = config.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = kCruiseVelocity;
        // motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
        // motionMagicConfigs.MotionMagicJerk = kJerk;
        // motionMagicConfigs.MotionMagicExpo_kV = kV;
        // motionMagicConfigs.MotionMagicExpo_kA = kA;

        config.MotorOutput.Inverted = kInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.ExternalFeedback.RotorToSensorRatio = (double)kDrivingGearTeeth/kSensingGearTeeth;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kUpperSoftLimit.in(Degrees);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kLowerSoftLimit.in(Degrees);

        config.Commutation.MotorArrangement = kMotorArrangement;

        m_motor.getConfigurator().apply(config);

        m_drivingEncoder = new DutyCycleEncoder(kDrivingEncoderID);
        m_sensingEncoder = new DutyCycleEncoder(kSensingEncoderID);

        m_temperature = m_motor.getDeviceTemp();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getTorqueCurrent();
        m_isPresent = () -> m_motor.isConnected() && !m_motor.getFault_HallSensorMissing().getValue();

        m_positionVoltage = new PositionVoltage(0).withEnableFOC(true);
        m_errorTolerance = kErrorTolerance;
    }

    @Override
    public void update()
    {
        StatusSignal.refreshAll(m_temperature, m_voltage, m_current);
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        m_targetAngle = angle;
        m_motor.setControl(m_positionVoltage.withPosition(angle.in(Rotations)));
    }

    @Override
    public void setSpeed(double speed)
    {
        m_motor.setControl(new DutyCycleOut(speed));
    }

    @Override
    public Angle getTargetAngle()
    {
        return m_targetAngle;
    }

    @Override
    public Angle getAngle()
    {
        return CRTEncodersUtil.calculateAngle(constants.kDrivingGearTeeth(), constants.kSensingGearTeeth(), constants.kTurntableGearTeeth(), Rotations.of(m_drivingEncoder.get()), Rotations.of(m_sensingEncoder.get()));
    }

    @Override
    public boolean isAtTargetAngle()
    {
        return Math.abs(getTargetAngle().in(Degrees) - getAngle().in(Degrees)) < m_errorTolerance.in(Degrees);
    }

    @Override
    public void resetAngle(Angle angle)
    {
        m_motor.setPosition(angle.in(Degrees));
    }

    @Override
    public String getAppliedControlName()
    {
        return m_motor.getAppliedControl().getName();
    }
}