package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOTalonFX implements IntakeIO
{
    private final TalonFXS rollerMotor;
    private final TalonFXS angleMotor;
    private final MotionMagicVoltage req;

    public IntakeIOTalonFX(int rollerMotorID, int angleMotorID, int encoderID)
    {
        this.rollerMotor = new TalonFXS(rollerMotorID);
        this.angleMotor = new TalonFXS(angleMotorID);
        this.req = new MotionMagicVoltage(0);

        var config = new TalonFXSConfiguration();
        config.ExternalFeedback.FeedbackRemoteSensorID = encoderID;
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotor.getConfigurator().apply(config);
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
        angleMotor.setControl(req.withPosition(angle));
    }

    @Override
    public void resetAngle()
    {
        angleMotor.setPosition(0);
    }
}
