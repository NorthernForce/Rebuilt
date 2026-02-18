package frc.robot.subsystems.fueldetection;

import edu.wpi.first.units.measure.Angle;

public interface FuelDetectorIO
{
    public boolean isFuelPresent();

    public Angle getFuelBlobXOffsets();

    public Angle getFuelBlobYOffsets();
}
