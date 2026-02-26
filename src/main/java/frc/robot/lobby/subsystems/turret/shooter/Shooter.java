package frc.robot.lobby.subsystems.turret.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase
{
    protected final ShooterIO io;
    protected final DoubleSubscriber m_targetVelocityOverride;
    protected final DoubleSubscriber m_targetDutyCycleOverride;

    public Shooter(ShooterIO io)
    {
        this.io = io;
        m_targetVelocityOverride = DogLog.tunable("Turret/Shooter/Target Velocity Override (RotationsPerSecond)",
                RotationsPerSecond.of(0.0), newVelocity ->
                {
                    setTargetSpeed(RotationsPerSecond.of(newVelocity));
                });
        m_targetDutyCycleOverride = DogLog.tunable("Turret/Shooter/Target Duty Cycle Override", 0.0, newDutyCycle ->
        {
            setTargetDutyCycle(newDutyCycle);
        });
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

    public void setTargetDutyCycle(double speed)
    {
        io.setTargetDutyCycle(speed);
    }

    public AngularVelocity getSpeed()
    {
        return io.getSpeed();
    }

    @Override
    public void periodic()
    {
        io.update();
        DogLog.log("Turret/Shooter/TargetSpeed", io.getTargetSpeed().in(RotationsPerSecond));
    }
}
