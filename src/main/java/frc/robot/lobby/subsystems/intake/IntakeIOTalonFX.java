package frc.robot.lobby.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Radians;
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
            Angle stowAngle)
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
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
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
        // Enforce software position limits defined in LobbyConstants.IntakeConstants
        double min = LobbyConstants.IntakeConstants.kStowedAngle.in(Radians);
        double max = LobbyConstants.IntakeConstants.kDownAngle.in(Radians);
        double reqRad = angle.in(Radians);
        double clamped = MathUtil.clamp(reqRad, min, max);
        if (clamped != reqRad)
        {
            // Log when clamping occurs
            SignalLogger.writeString("IntakeArm/Limit", "Clamped request from " + reqRad + " to " + clamped);
        }
        angleMotor.setControl(req.withPosition(Radians.of(clamped)));
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
        SignalLogger.writeDouble("IntakeArm/Position", angleMotor.getPosition().getValueAsDouble());
        SignalLogger.writeDouble("IntakeArm/Velocity", angleMotor.getVelocity().getValueAsDouble());
        SignalLogger.writeDouble("IntakeArm/Voltage", angleMotor.getMotorVoltage().getValueAsDouble());
    }
}
