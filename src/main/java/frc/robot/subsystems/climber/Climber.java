package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.command.ExtendClimber;
import frc.robot.subsystems.climber.command.RetractClimber;

@Logged
public class Climber extends SubsystemBase
{
    private final int m_motorId;
    private final TalonFX m_motor;
    private final double m_climbSpeed;
    private final double gearRatio;
    private final Angle lowerLimit;
    private final Angle upperLimit;

    private ClimberState climberState = ClimberState.STOP;

    public Climber(int motorId, double climbSpeed, boolean inverted, double gearRatio, Angle lowerLimit,
            Angle upperLimit)
    {
        m_motorId = motorId;
        m_motor = new TalonFX(m_motorId);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = (inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_motor.getConfigurator().apply(config);
        m_climbSpeed = climbSpeed;

        this.gearRatio = gearRatio;
        this.lowerLimit = lowerLimit;
        this.upperLimit = upperLimit;
    }

    public void stop()
    {
        m_motor.stopMotor();
    }

    public void extend()
    {
        m_motor.set(m_climbSpeed);
    }

    public ClimberState getState()
    {
        return climberState;
    }

    public void setState(ClimberState climberState)
    {
        this.climberState = climberState;
    }

    public void retract()
    {
        m_motor.set(-m_climbSpeed);
    }

    public Angle getRotations()
    {
        Angle rotorPosRotations = m_motor.getPosition().getValue();
        return rotorPosRotations;
    }

    public Angle getAngle()
    {
        return Units.Rotations.of(getRotations().in(Units.Rotations) / gearRatio);
    }

    public Command extendClimber()
    {
        return new ExtendClimber(this);
    }

    public Command retractClimber()
    {
        return new RetractClimber(this);
    }

    @Override
    public void periodic()
    {

        if (getAngle().compareTo(lowerLimit) < 0)
        {
            climberState = ClimberState.STOP;
        }

        if (getAngle().compareTo(upperLimit) > 0)
        {
            climberState = ClimberState.STOP;
        }

        switch (climberState)
        {
        case STOP:
            stop();
            break;
        case EXTEND:
            extend();
            break;
        case RETRACT:
            retract();
            break;
        default:
            stop();
            break;
        }
    }

    public static enum ClimberState
    {
        EXTEND, RETRACT, STOP
    }

}