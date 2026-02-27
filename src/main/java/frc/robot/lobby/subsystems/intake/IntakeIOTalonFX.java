package frc.robot.lobby.subsystems.intake;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

public class IntakeIOTalonFX implements IntakeIO
{
    private final TalonFXS rollerMotor;
    private final TalonFXS angleMotor;
    private final MotionMagicVoltage req;
    private final VoltageOut voltageReq = new VoltageOut(0);

    private final double forwardSoftLimit;
    private final double reverseSoftLimit;

    public IntakeIOTalonFX(IntakeIOParameters intakeParams)
    {
        this.rollerMotor = new TalonFXS(intakeParams.rollerMotorID());
        this.angleMotor = new TalonFXS(intakeParams.angleMotorID());
        this.req = new MotionMagicVoltage(0);

        var config = new TalonFXSConfiguration();
        config.ExternalFeedback.FeedbackRemoteSensorID = intakeParams.encoderID();
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = intakeParams.forwardSoftLimit();
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = intakeParams.reverseSoftLimit();
        forwardSoftLimit = intakeParams.forwardSoftLimit();
        reverseSoftLimit = intakeParams.reverseSoftLimit();
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.Slot0.kS = intakeParams.kS();
        config.Slot0.kP = intakeParams.kP();
        config.Slot0.kI = intakeParams.kI();
        config.Slot0.kD = intakeParams.kD();
        config.Slot0.kG = intakeParams.kG();
        config.Slot0.kV = intakeParams.kV();
        config.Slot0.kA = intakeParams.kA();
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotionMagic.MotionMagicExpo_kV = intakeParams.kV();
        config.MotionMagic.MotionMagicExpo_kA = intakeParams.kA();
        config.MotionMagic.MotionMagicAcceleration = intakeParams.acceleration();
        config.MotionMagic.MotionMagicCruiseVelocity = intakeParams.cruiseVelocity();
        config.CurrentLimits.StatorCurrentLimit = intakeParams.currentLimit().in(Amps);
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        angleMotor.getConfigurator().apply(config);

        var rollerConfig = new TalonFXSConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.CurrentLimits.StatorCurrentLimit = intakeParams.currentLimit().in(Amps);
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerMotor.getConfigurator().apply(rollerConfig);

        // Optimize signal frequencies for SysId logging
        angleMotor.getPosition().setUpdateFrequency(250);
        angleMotor.getVelocity().setUpdateFrequency(250);
        angleMotor.getMotorVoltage().setUpdateFrequency(250);
    }

    @Override
    public void intake(double speed)
    {
        rollerMotor.set(MathUtil.clamp(speed, 0.0, 1.0));
    }

    @Override
    public void purgeIntake(double speed)
    {
        rollerMotor.set(-MathUtil.clamp(speed, 0.0, 1.0));
    }

    @Override
    public void stopIntake()
    {
        rollerMotor.set(0);
    }

    @Override
    public void setAngle(Angle angle)
    {
        angleMotor.setControl(req.withPosition(angle.in(Rotations)));
    }

    @Override
    public void resetAngle()
    {
        angleMotor.setPosition(0);
    }

    @Override
    public void setArmVoltage(Voltage voltage)
    {
        angleMotor.setControl(voltageReq.withOutput(voltage));
    }

    @Override
    public void logArmSignals()
    {
        DogLog.log("IntakeArm/Position", angleMotor.getPosition().getValueAsDouble());
        DogLog.log("IntakeArm/Velocity", angleMotor.getVelocity().getValueAsDouble());
        DogLog.log("IntakeArm/Voltage", angleMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public double getArmPosition()
    {
        return angleMotor.getPosition().getValueAsDouble();
    }

    @Override
    public double getArmVelocity()
    {
        return angleMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getArmVoltage()
    {
        return angleMotor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void disableSoftLimits()
    {
        var config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = false;
        config.ReverseSoftLimitEnable = false;
        angleMotor.getConfigurator().apply(config);
    }

    @Override
    public void enableSoftLimits()
    {
        var config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitThreshold = forwardSoftLimit;
        config.ReverseSoftLimitThreshold = reverseSoftLimit;
        config.ForwardSoftLimitEnable = true;
        config.ReverseSoftLimitEnable = true;
        angleMotor.getConfigurator().apply(config);
    }

    @Override
    public void setPower(double power)
    {
        angleMotor.set(power);
    }
}
