package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import java.io.ObjectInputFilter.Status;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.TunablePID;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class SuzieIOTalonFXS implements SuzieIO
{
    protected SuzieConstants constants;
    protected final TalonFXS m_motor;
    protected DutyCycleEncoder m_drivingEncoder;
    protected DutyCycleEncoder m_sensingEncoder;
    protected final StatusSignal<Angle> m_position;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final StatusSignal<AngularVelocity> m_velocity;
    protected final Supplier<Boolean> m_isPresent;
    protected final MotionMagicExpoVoltage m_positionVoltage;
    protected final Angle m_errorTolerance;
    protected final EasyCRT m_crtCalculator;
    protected final Angle m_lowerSoftLimit;
    protected final Angle m_upperSoftLimit;
    protected final StatusSignal<Current> motorCurrent;

    public boolean crtUsed = false;
    private Angle m_targetAngle = Degrees.zero();

    public SuzieIOTalonFXS(SuzieIO.SuzieConstants constants)
    {
        this(constants.kMotorID(), constants.kDrivingEncoderID(), constants.kSensingEncoderID(), constants.kS(),
                constants.kV(), constants.kA(), constants.kP(), constants.kI(), constants.kD(), constants.kG(),
                constants.kCruiseVelocity(), constants.kAcceleration(), constants.kJerk(), constants.kExpoV(),
                constants.kExpoA(), constants.kRotorToTurntableRatio(), constants.kDrivingGearTeeth(),
                constants.kSensingGearTeeth(), constants.kTurntableGearTeeth(), constants.kInverted(),
                constants.kLowerSoftLimit(), constants.kUpperSoftLimit(), constants.kErrorTolerance(),
                constants.kMotorArrangement()/* , constants.kCRTConfig() */);
        this.constants = constants;
    }

    private SuzieIOTalonFXS(int kMotorID, int kDrivingEncoderID, int kSensingEncoderID, double kS, double kV, double kA,
            double kP, double kI, double kD, double kG, double kCruiseVelocity, double kAcceleration, double kJerk,
            double kExpoV, double kExpoA, double kRotorToTurntableRatio, int kDrivingGearTeeth, int kSensingGearTeeth,
            int kTurntableGearTeeth, boolean kInverted, Angle kLowerSoftLimit, Angle kUpperSoftLimit,
            Angle kErrorTolerance, MotorArrangementValue kMotorArrangement/* , EasyCRTConfig kCRTConfig */)
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
        motionMagicConfigs.MotionMagicExpo_kV = kExpoV;
        motionMagicConfigs.MotionMagicExpo_kA = kExpoA;

        config.ExternalFeedback.SensorToMechanismRatio = kRotorToTurntableRatio;

        config.ExternalFeedback.SensorToMechanismRatio = kRotorToTurntableRatio;

        config.MotorOutput.Inverted = kInverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kUpperSoftLimit.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kLowerSoftLimit.in(Rotations);

        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.Commutation.MotorArrangement = kMotorArrangement;

        m_motor.getConfigurator().apply(config);

        m_drivingEncoder = new DutyCycleEncoder(kDrivingEncoderID, 1.0,
                Preferences.getDouble("drivingEncoderOffset", 0));
        m_sensingEncoder = new DutyCycleEncoder(kSensingEncoderID, 1.0,
                Preferences.getDouble("sensingEncoderOffset", 0));

        m_position = m_motor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getTorqueCurrent();
        m_velocity = m_motor.getVelocity();
        m_isPresent = () -> m_motor.isConnected() && !m_motor.getFault_HallSensorMissing().getValue();

        m_positionVoltage = new MotionMagicExpoVoltage(0).withEnableFOC(true);
        m_errorTolerance = kErrorTolerance;

        m_lowerSoftLimit = kLowerSoftLimit;
        m_upperSoftLimit = kUpperSoftLimit;

        EasyCRTConfig crtConfig = new EasyCRTConfig(() -> Rotations.of(m_drivingEncoder.get()),
                () -> Rotations.of(m_sensingEncoder.get()))
                .withAbsoluteEncoder1Gearing(kTurntableGearTeeth, kDrivingGearTeeth)
                .withAbsoluteEncoder2Gearing(kTurntableGearTeeth, kSensingGearTeeth)
                .withMechanismRange(Rotations.of(-0.55), Rotations.of(0.55)).withMatchTolerance(Degrees.of(5))
                .withAbsoluteEncoder1Inverted(true).withAbsoluteEncoder2Inverted(true);
        m_crtCalculator = new EasyCRT(crtConfig);

        TunablePID.createMotionMagic("Turret/Suzie/PID", m_motor, config);
        motorCurrent = m_motor.getSupplyCurrent();
    }

    @Override
    public void update()
    {
        StatusSignal.refreshAll(m_position, m_temperature, m_voltage, m_current, m_velocity);

        var angle = m_crtCalculator.getAngleOptional();
        if (angle.isPresent())
        {
            if ((crtUsed && angle.get().isNear(m_motor.getPosition().getValue(), Degrees.of(10))) || !crtUsed)
            {
                m_motor.setPosition(angle.get());
                crtUsed = true;
            }
        }

        DogLog.log("Turret/Suzie/CRT Used", crtUsed);
        DogLog.log("Turret/Suzie/CRT Status", m_crtCalculator.getLastStatus().toString());
        DogLog.log("Turret/Suzie/Driving Encoder Position", m_drivingEncoder.get());
        DogLog.log("Turret/Suzie/Sensing Encoder Position", m_sensingEncoder.get());
    }

    @Override
    public void setMotorControl(ControlRequest request)
    {
        m_motor.setControl(request);
    }

    @Override
    public void resetCRT()
    {
        crtUsed = false;
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        m_targetAngle = angle;
    }

    @Override
    public void start()
    {
        m_motor.setControl(m_positionVoltage.withPosition(m_targetAngle.in(Rotations)));
    }

    @Override
    public void stop()
    {
        m_motor.stopMotor();
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
        return m_motor.getPosition().getValue();
    }

    @Override
    public boolean isAtTargetAngle()
    {
        return getAngle().isNear(getTargetAngle(), m_errorTolerance);
    }

    @Override
    public void resetAngle(Angle angle)
    {
        m_motor.setPosition(angle.in(Degrees));
    }

    @Override
    public Voltage getVoltage()
    {
        return m_voltage.getValue();
    }

    @Override
    public AngularVelocity getVelocity()
    {
        return m_velocity.getValue();
    }

    @Override
    public void disableSoftLimits()
    {
        m_motor.getConfigurator().apply(
                new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(false).withReverseSoftLimitEnable(false));
    }

    @Override
    public void enableSoftLimits()
    {
        m_motor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(m_upperSoftLimit).withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(m_lowerSoftLimit));
    }

    @Override
    public String getAppliedControlName()
    {
        return m_motor.getAppliedControl().getName();
    }

    @Override
    public double getCurrent()
    {
        motorCurrent.refresh();
        return motorCurrent.getValueAsDouble();
    }

    @Override
    public void setDrivingEncoderOffset(Angle angle)
    {
        m_drivingEncoder.close();
        m_drivingEncoder = new DutyCycleEncoder(constants.kDrivingEncoderID(), 1.0, angle.in(Rotations));
    }

    @Override
    public void setSensingEncoderOffset(Angle angle)
    {
        m_sensingEncoder.close();
        m_sensingEncoder = new DutyCycleEncoder(constants.kSensingEncoderID(), 1.0, angle.in(Rotations));
    }

    @Override
    public Angle getDrivingEncoderAngle()
    {
        return Rotations.of(m_drivingEncoder.get());
    }

    @Override
    public Angle getSensingEncoderAngle()
    {
        return Rotations.of(m_sensingEncoder.get());
    }
}