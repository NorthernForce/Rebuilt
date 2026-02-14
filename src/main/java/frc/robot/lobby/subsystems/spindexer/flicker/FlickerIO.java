package frc.robot.lobby.subsystems.spindexer.flicker;

public interface FlickerIO
{
    public void rampFlicker();

    public void stopFlicker();

    public boolean isAtTargetSpeed();

    /** Called every loop to update simulation physics. No-op on real robot. */
    public default void updateSimulation(double dtSeconds)
    {
    }
}
