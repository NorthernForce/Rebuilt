package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.util.TunablePID;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class SuzieIOTalonFXS implements SuzieIO
{
    protected SuzieConstants constants;
    protected final TalonFXS m_motor;
    protected Encoder m_sensingEncoder;
    protected final StatusSignal<Angle> m_position;
    protected final StatusSignal<Temperature> m_temperature;
    protected final StatusSignal<Voltage> m_voltage;
    protected final StatusSignal<Current> m_current;
    protected final StatusSignal<AngularVelocity> m_velocity;
    protected final Supplier<Boolean> m_isPresent;
    protected final PositionVoltage m_positionVoltage;
    protected final Angle m_errorTolerance;
    protected final Angle m_lowerSoftLimit;
    protected final Angle m_upperSoftLimit;
    protected final StatusSignal<Current> motorCurrent;
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

        // var motionMagicConfigs = config.MotionMagic;
        // motionMagicConfigs.MotionMagicCruiseVelocity = kCruiseVelocity;
        // motionMagicConfigs.MotionMagicAcceleration = kAcceleration;
        // motionMagicConfigs.MotionMagicJerk = kJerk;
        // motionMagicConfigs.MotionMagicExpo_kV = kExpoV;
        // motionMagicConfigs.MotionMagicExpo_kA = kExpoA;

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

        m_sensingEncoder = new Encoder(8, 7);
        m_sensingEncoder.setSamplesToAverage(5);
        m_sensingEncoder.setDistancePerPulse(1.0 / 2048.0);
        m_sensingEncoder.setMinRate(1.0);
        // kSensingEncoderID, 1.0,
        // Preferences.getDouble("sensingEncoderOffset", 0));

        m_position = m_motor.getPosition();
        m_temperature = m_motor.getDeviceTemp();
        m_voltage = m_motor.getMotorVoltage();
        m_current = m_motor.getTorqueCurrent();
        m_velocity = m_motor.getVelocity();
        m_isPresent = () -> m_motor.isConnected() && !m_motor.getFault_HallSensorMissing().getValue();

        m_positionVoltage = new PositionVoltage(0).withEnableFOC(true);
        m_errorTolerance = kErrorTolerance;

        m_lowerSoftLimit = kLowerSoftLimit;
        m_upperSoftLimit = kUpperSoftLimit;

        TunablePID.createBasic("Turret/Suzie/PID", m_motor, config);
        motorCurrent = m_motor.getSupplyCurrent();
        m_motor.setPosition(0.0);
    }

    @Override
    public void update()
    {
        m_motor.setPosition(Rotations
                .of(m_sensingEncoder.getDistance() * constants.kSensingGearTeeth() / constants.kTurntableGearTeeth()));
        StatusSignal.refreshAll(m_position, m_temperature, m_voltage, m_current, m_velocity);
        DogLog.log("Turret/Suzie/Sensing Encoder Position", m_sensingEncoder.getDistance());
    }

    @Override
    public void setMotorControl(ControlRequest request)
    {
        m_motor.setControl(request);
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        // if (!m_targetAngle.isNear(angle, m_errorTolerance))
        m_targetAngle = angle;
    }

    @Override
    public void start()
    {
        m_motor.setControl(m_positionVoltage.withPosition(m_targetAngle.in(Rotations)));
    }

    @Override
    public void start(double chassisOmegaRadPerSec)
    {
        double turretVelocityRotPerSec = -chassisOmegaRadPerSec / (2.0 * Math.PI);
        double ffVoltage = constants.kV() * turretVelocityRotPerSec;
        m_motor.setControl(m_positionVoltage.withPosition(m_targetAngle.in(Rotations)).withFeedForward(ffVoltage));
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
    public void resetAngle()
    {
        m_motor.setPosition(0.0);
        m_sensingEncoder.reset();
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
    public Slot0Configs getSlot0Configs()
    {
        var config = new TalonFXSConfiguration();
        m_motor.getConfigurator().refresh(config);
        return config.Slot0;
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
    public Angle getSensingEncoderAngle()
    {
        return Rotations.of(m_sensingEncoder.getDistance());
    }

    @Override
    public void setBrakeMode(boolean brake)
    {
        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        m_motor.getConfigurator().refresh(outputConfigs);
        outputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_motor.getConfigurator().apply(outputConfigs);
    }
}