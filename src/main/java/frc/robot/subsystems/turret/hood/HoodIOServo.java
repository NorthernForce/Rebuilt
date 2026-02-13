package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO
{
    private final Servo m_motor;
    private final Angle m_lowerSoftLimit;
    private final Angle m_upperSoftLimit;

    private Angle m_targetAngle;

    public HoodIOServo(HoodConstants constants)
    {
        m_motor = new Servo(constants.kMotorID());
        m_lowerSoftLimit = constants.kLowerSoftLimit();
        m_upperSoftLimit = constants.kUpperSoftLimit();
        m_targetAngle = m_lowerSoftLimit;
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        if (angle.lt(m_lowerSoftLimit))
        {
            angle = m_lowerSoftLimit;
        } else if (angle.gt(m_upperSoftLimit))
        {
            angle = m_upperSoftLimit;
        }
        m_targetAngle = angle;
        m_motor.setAngle(angle.in(Degrees));
    }

    @Override
    public Angle getTargetAngle()
    {
        return m_targetAngle;
    }

    @Override
    public boolean isAtTargetAngle()
    {
        return true;
    }
}
