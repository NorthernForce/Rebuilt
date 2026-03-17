package frc.robot.lobby.subsystems.spindexer.flicker;

import edu.wpi.first.wpilibj.PowerDistribution;

public interface FlickerIO
{
    public void rampFlicker();

    public void stopFlicker();

    /** Called every loop to update simulation physics. No-op on real robot. */
    public default void updateSimulation(double dtSeconds)
    {
    }

    public double getPower();

    public void setPower(double power);

    public double getTargetPower();

    public boolean getJammed();

    public void dejam();

    public void resetJamDetection();

    public double getCurrent();

}
