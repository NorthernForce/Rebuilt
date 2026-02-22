package frc.robot.subsystems.turret.hood;

import java.util.List;

import edu.wpi.first.math.geometry.Transform2d;
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

    public List<Translation2d> getTrenchPositions()
    {
        return io.getTrenchPositions();
    }

    public Distance getDangerZone()
    {
        return io.getDangerZone();
    }

    public Angle getTargetAngle()
    {
        return io.getTargetAngle();
    }

    public Angle getAngle()
    {
        return io.getAngle();
    }

    public boolean isAtTargetAngle()
    {
        return io.isAtTargetAngle();
    }

    public HoodIO getIO()
    {
        return io;
    }

    @Override
    public void periodic()
    {
        io.update();
    }
}
