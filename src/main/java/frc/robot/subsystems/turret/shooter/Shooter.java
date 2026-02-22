package frc.robot.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
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

    public AngularVelocity getTargetSpeed()
    {
        return io.getTargetSpeed();
    }

    public Command start()
    {
        return run(() -> io.start());
    }

    public Command stop()
    {
        return run(() -> io.stop());
    }

    public void setTargetSpeed(AngularVelocity speed)
    {
        io.setTargetSpeed(speed);
    }

    public AngularVelocity getSpeed()
    {
        return io.getSpeed();
    }

    public void setPotentialSpeed(AngularVelocity speed)
    {
        io.setPotentialSpeed(speed);
    }

    public void setPID(double p, double i, double d, double v, double a)
    {
        io.setPID(p, i, d, v, a);
    }

    @Override
    public void periodic()
    {
        io.update();
        DogLog.log("Turret/Shooter/TargetSpeed", io.getTargetSpeed().in(RotationsPerSecond));
    }
}
