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

    @Override
    public void periodic()
    {

        DogLog.log("Turret/Suzie/ControlRequest", io.getAppliedControlName());
    }

}
