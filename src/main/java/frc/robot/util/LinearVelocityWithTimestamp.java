package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class LinearVelocityWithTimestamp
{
    private LinearVelocity velocity;
    private Time timestamp;

    public LinearVelocityWithTimestamp(LinearVelocity velocity, Time timestamp)
    {
        this.velocity = velocity;
        this.timestamp = timestamp;
    }

    public LinearVelocityWithTimestamp(LinearVelocity velocity)
    {
        this.velocity = velocity;
        timestamp = Seconds.of(Timer.getFPGATimestamp());
    }

    public LinearVelocity getVelocity()
    {
        return velocity;
    }

    public Time getTime()
    {
        return timestamp;
    }
}
