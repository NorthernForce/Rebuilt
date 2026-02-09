package frc.robot.subsystems.flicker;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class FlickerIOTalonFXS implements FlickerIO
{
    private int m_motorId;
    private TalonFXS m_motor;
    private double m_targetSpeed = 0.0;
    private double m_tolerance;

    public FlickerIOTalonFXS(int motorId, double tolerance)
    {
        m_motorId = motorId;
        m_tolerance = tolerance;
        m_motor = new TalonFXS(m_motorId);
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(config);
    }

    @Override
    public void rampFlicker(double speed)
    {
        m_targetSpeed = speed;
        m_motor.set(speed);
    }

    @Override
    public void stopFlicker()
    {
        m_motor.set(0.0);
    }

    @Override
    public double getFlickerSpeed()
    {
        return MathUtil.clamp(m_motor.getDutyCycle().getValue() / 2.0, -1.0, 1.0);
    }

    @Override
    public boolean flickerIsAtSpeed()
    {
        return Math.abs(getFlickerSpeed() - m_targetSpeed) < m_tolerance;
    }

    @Override
    public Command getRampCommand(double speed)
    {
        return Commands.runOnce(() -> rampFlicker(speed));
    }

    @Override
    public Command getStopCommand()
    {
        return Commands.runOnce(() -> stopFlicker());
    }
}
