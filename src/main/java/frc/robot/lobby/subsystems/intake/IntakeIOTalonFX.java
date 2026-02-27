package frc.robot.lobby.subsystems.intake;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import frc.robot.lobby.LobbyConstants;

public class IntakeIOTalonFX implements IntakeIO
{
    private final TalonFXS rollerMotor;
    private final TalonFXS angleMotor;
    private final MotionMagicVoltage req;
    private final VoltageOut voltageReq = new VoltageOut(0);
    private final Angle downAngle;
    private final Angle midAngle;
    private final Angle stowAngle;

    public IntakeIOTalonFX(int rollerMotorID, int angleMotorID, int encoderID, Angle downAngle, Angle midAngle,
            Angle stowAngle, double kP, double kI, double kD, double kS, double kV, double kA, double kG)
    {
        this.rollerMotor = new TalonFXS(rollerMotorID);
        this.angleMotor = new TalonFXS(angleMotorID);
        this.req = new MotionMagicVoltage(0);
        this.downAngle = downAngle;
        this.midAngle = midAngle;
        this.stowAngle = stowAngle;
        var config = new TalonFXSConfiguration();
        config.ExternalFeedback.FeedbackRemoteSensorID = encoderID;
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = stowAngle.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = downAngle.in(Rotations);
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.Slot0.kS = kS;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kG = kG;
        config.MotionMagic.MotionMagicExpo_kV = kV;
        config.MotionMagic.MotionMagicExpo_kA = kA;

        angleMotor.getConfigurator().apply(config);

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
    public void runToStowAngle()
    {
        setAngle(stowAngle);
    }

    @Override
    public void runToMidAngle()
    {
        setAngle(midAngle);
    }

    @Override
    public void runToIntakeAngle()
    {
        setAngle(downAngle);
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
        config.ForwardSoftLimitThreshold = stowAngle.in(Rotations);
        config.ReverseSoftLimitThreshold = downAngle.in(Rotations);
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
