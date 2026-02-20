package frc.robot.subsystems.turret.hood;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase
{
    protected final HoodIO io;

    public Hood(HoodIO io)
    {
        this.io = io;
    }

    public Command setTargetAngle(Angle angle)
    {
        return run(() -> io.setTargetAngle(angle));
    }

    public Command setTargetMechanismAngle(Angle angle)
    {
        return run(() -> io.setTargetMechanismAngle(angle));
    }

    public Command setSpeed(double speed, boolean overrideSoftLimit)
    {
        return run(() -> io.setSpeed(speed, overrideSoftLimit));
    }

    public HoodIO getIO()
    {
        return io;
    }
}
