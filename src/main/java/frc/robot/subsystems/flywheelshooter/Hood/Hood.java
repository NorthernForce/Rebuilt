package frc.robot.subsystems.flywheelshooter.Hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase
{
    private final HoodIO io;

    public Hood(HoodIO io)
    {
        this.io = io;
    }

    public void setHoodAngle(Angle angle)
    {
        io.setHoodAngle(angle);
    }

    public Angle getHoodAngle()
    {
        return io.getHoodAngle();
    }

    public Angle getTargetAngle()
    {
        return io.getTargetAngle();
    }

    public boolean atTargetAngle()
    {
        return io.atTargetAngle();
    }

    public void resetAngle()
    {
        io.resetAngle();
    }

}
