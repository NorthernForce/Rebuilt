package frc.robot.lobby.subsystems.flicker;

public interface FlickerIO
{
    public void rampFlicker();

    public void stopFlicker();

    public double getSpeed();

    /** Called every loop to update simulation physics. No-op on real robot. */
    public default void updateSimulation(double dtSeconds)
    {
    }
}
