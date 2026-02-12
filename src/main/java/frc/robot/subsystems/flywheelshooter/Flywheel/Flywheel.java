package frc.robot.subsystems.flywheelshooter.Flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase
{
    private final FlywheelIO io;

    public Flywheel(FlywheelIO io)
    {
        this.io = io;
    }

    public void setFlywheelSpeed(double speed)
    {
        io.run(speed);
    }

    public void stopFlywheel()
    {
        io.stop();
    }

    public double getFlywheelSpeed()
    {
        return io.getSpeed();
    }

    public double getFlywheelTargetSpeed()
    {
        return io.getTargetSpeed();
    }

}