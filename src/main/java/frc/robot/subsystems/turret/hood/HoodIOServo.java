package frc.robot.subsystems.turret.hood;

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
    private final Servo m_motor;
    private final Angle m_lowerSoftLimit;
    private final Angle m_upperSoftLimit;
    private final Distance m_dangerZone;
    private final List<Translation2d> m_trenchPositions;
    private final Angle m_lowerMechanismAngle;
    private final Angle m_upperMechanismAngle;

    private Angle m_targetAngle;
    private Angle m_targetMechanicalAngle;

    public HoodIOServo(HoodConstants constants)
    {
        m_motor = new Servo(constants.kMotorID());
        m_lowerSoftLimit = constants.kLowerSoftLimit();
        m_upperSoftLimit = constants.kUpperSoftLimit();
        m_targetAngle = m_lowerSoftLimit;
        m_dangerZone = constants.dangerZone();
        m_trenchPositions = constants.trenchPositions();
        m_lowerMechanismAngle = constants.kLowerMechanismAngle();
        m_upperMechanismAngle = constants.kUpperMechanismAngle();
        m_targetMechanicalAngle = m_lowerMechanismAngle;

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
            m_motor.setAngle(angle.in(Degrees));
        }
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

        m_targetMechanicalAngle = targetServoAngle;
        m_motor.setAngle(m_targetMechanicalAngle.in(Degrees));
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
