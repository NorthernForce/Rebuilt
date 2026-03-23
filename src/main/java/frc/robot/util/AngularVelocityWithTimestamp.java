package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;

public class AngularVelocityWithTimestamp
{
    private AngularVelocity velocity;
    private Time timestamp;

    public AngularVelocityWithTimestamp(AngularVelocity velocity, Time timestamp)
    {
        this.velocity = velocity;
        this.timestamp = timestamp;
    }

    public AngularVelocityWithTimestamp(AngularVelocity velocity)
    {
        this.velocity = velocity;
        timestamp = Seconds.of(Timer.getFPGATimestamp());
    }

    public AngularVelocity getVelocity()
    {
        return velocity;
    }

    public Time getTime()
    {
        return timestamp;
    }
}
