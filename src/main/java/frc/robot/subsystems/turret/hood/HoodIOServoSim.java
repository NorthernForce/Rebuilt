package frc.robot.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.turret.hood.HoodIO.HoodConstants;

public class HoodIOServoSim extends HoodIOServo
{
    private Angle m_simulatedAngle;

    public HoodIOServoSim(HoodConstants constants)
    {
        super(constants);
        m_simulatedAngle = constants.kLowerSoftLimit();
    }

    @Override
    public void setTargetAngle(Angle angle)
    {
        super.setTargetAngle(angle);
        m_simulatedAngle = getTargetAngle();
    }

    @Override
    public Angle getAngle()
    {
        return m_simulatedAngle;
    }
}
