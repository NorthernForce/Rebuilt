package frc.robot.subsystems.turret.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    protected final ShooterIO io;

    public Shooter(ShooterIO io)
    {
        this.io = io;
    }

    public ShooterIO getIO()
    {
        return io;
    }

    public Command start()
    {
        return run(() -> io.start());
    }

    public Command stop()
    {
        return run(() -> io.stop());
    }
}
