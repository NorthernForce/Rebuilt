package frc.robot.lobby.subsystems.turret.suzie;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Suzie extends SubsystemBase
{
    protected final SuzieIO io;
    protected final DoubleSubscriber m_targetAngleOverride;

    public Suzie(SuzieIO io)
    {
        this.io = io;
        m_targetAngleOverride = DogLog.tunable("Turret/Suzie/Target Angle Override (Degrees)", Degrees.of(0.0),
                newAngle ->
                {
                    setTargetAngle(Degrees.of(newAngle));
                });
    }

    public Command setSpeed(double speed)
    {
        return run(() -> io.setSpeed(speed));
    }

    public Command start()
    {
        return run(() -> io.start());
    }

    public Command stop()
    {
        return run(() -> io.stop());
    }

    public SuzieIO getIO()
    {
        return io;
    }

    public Angle getAngle()
    {
        return io.getAngle();
    }

    public Angle getTargetAngle()
    {
        return io.getTargetAngle();
    }

    public boolean isAtTargetAngle()
    {
        return io.isAtTargetAngle();
    }

    public void setTargetAngle(Angle angle)
    {
        io.setTargetAngle(angle);
    }

    @Override
    public void periodic()
    {
        io.update();
    }
}
