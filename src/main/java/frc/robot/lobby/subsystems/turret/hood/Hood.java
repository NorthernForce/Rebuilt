package frc.robot.lobby.subsystems.turret.hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase
{
    protected final HoodIO io;
    protected DoubleSubscriber m_targetMechanismAngleOverride;

    public Hood(HoodIO io)
    {
        this.io = io;
        m_targetMechanismAngleOverride = DogLog.tunable("Turret/Hood/Target Mechanism Angle Override (Degrees)",
                io.getLowerMechanismLimit(), newAngle ->
                {
                    setTargetAngle(Degrees.of(newAngle));
                });
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

    public Command start()
    {
        return run(() -> io.start());
    }

    public Command stop()
    {
        return run(() -> io.stop());
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
