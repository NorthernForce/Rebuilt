package frc.robot.lobby.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO
{
    protected final Servo m_motor;
    protected final Angle m_lowerSoftLimit;
    protected final Angle m_upperSoftLimit;
    protected final Distance m_dangerZone;
    protected final List<Translation2d> m_trenchPositions;
    protected final Angle m_lowerMechanismAngle;
    protected final Angle m_upperMechanismAngle;

    protected Angle m_targetAngle;

    public HoodIOServo(HoodConstants constants)
    {
        m_motor = new Servo(constants.kMotorID());
        m_lowerSoftLimit = constants.kLowerSoftLimit();
        m_upperSoftLimit = constants.kUpperSoftLimit();
        m_targetAngle = m_lowerSoftLimit;
        m_dangerZone = constants.kDangerZone();
        m_trenchPositions = constants.trenchPositions();
        m_lowerMechanismAngle = constants.kLowerMechanismAngle();
        m_upperMechanismAngle = constants.kUpperMechanismAngle();

    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        if (angle.in(Degrees) != m_targetAngle.in(Degrees))
        {
            DogLog.log("Turret/Hood/MechanismAngle",
                    (m_lowerSoftLimit.minus(angle).div(m_lowerSoftLimit.minus(m_upperSoftLimit))
                            .times(m_upperMechanismAngle.minus(m_lowerMechanismAngle)).plus(m_lowerMechanismAngle)));
            DogLog.log("Turret/Hood/SetTargetAngle", angle.in(Radians));
            if (angle.lt(m_lowerSoftLimit))
            {
                angle = m_lowerSoftLimit;
            } else if (angle.gt(m_upperSoftLimit))
            {
                angle = m_upperSoftLimit;
            }
            m_targetAngle = angle;
        }
    }

    @Override
    public Angle getLowerMechanismLimit()
    {
        return m_lowerMechanismAngle;
    }

    @Override
    public void start()
    {
        m_motor.setAngle(m_targetAngle.in(Degrees));
    }

    @Override
    public void stop()
    {
        m_motor.setSpeed(0);
    }

    @Override
    public void setTargetMechanismAngle(Angle angle)
    {
        Angle targetServoAngle = angle.minus(m_lowerMechanismAngle)
                .div(m_upperMechanismAngle.minus(m_lowerMechanismAngle)).times(m_lowerSoftLimit.minus(m_upperSoftLimit))
                .plus(m_lowerSoftLimit);
        if (targetServoAngle.gt(m_lowerSoftLimit))
        {
            targetServoAngle = m_lowerSoftLimit;
        } else if (targetServoAngle.lt(m_upperSoftLimit))
        {
            targetServoAngle = m_upperSoftLimit;
        }

        m_targetAngle = targetServoAngle;
        m_motor.setAngle(m_targetAngle.in(Degrees));
    }

    @Override
    public Angle getTargetAngle()
    {
        return m_targetAngle;
    }

    @Override
    public Angle getAngle()
    {
        return m_targetAngle;
    }

    @Override
    public Distance getDangerZone()
    {
        return m_dangerZone;
    }

    @Override
    public List<Translation2d> getTrenchPositions()
    {
        return m_trenchPositions;
    }

    @Override
    public boolean isAtTargetAngle()
    {
        return true;
    }
}
