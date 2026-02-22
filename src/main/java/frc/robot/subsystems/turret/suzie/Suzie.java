package frc.robot.subsystems.turret.suzie;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Suzie extends SubsystemBase
{
    protected final SuzieIO io;

    public Suzie(SuzieIO io)
    {
        this.io = io;
    }

    public Command setSpeed(double speed)
    {
        return run(() -> io.setSpeed(speed));
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
        DogLog.log("Turret/Suzie/ControlRequest", io.getAppliedControlName());
    }

}
