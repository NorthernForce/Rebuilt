package frc.robot.subsystems.fueldetection;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelDetector extends SubsystemBase
{
    public FuelDetectorIO io;

    public FuelDetector(FuelDetectorIO io)
    {
        this.io = io;
    }

    public boolean isFuelPresent()
    {
        return io.isFuelPresent();
    }

    public Angle getFuelBlobXOffsets()
    {
        return io.getFuelBlobXOffsets();
    }

    public Angle getFuelBlobYOffsets()
    {
        return io.getFuelBlobYOffsets();
    }
}
