package frc.robot.subsystems.fueldetection;

import static edu.wpi.first.units.Units.Degrees;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import frc.robot.LimelightHelpers;

public class FuelDetectorIOLimelight implements FuelDetectorIO
{

    private final String limelightName;
    private final NetworkTable table;

    public FuelDetectorIOLimelight(String limelightName)
    {
        this.limelightName = limelightName;
        this.table = NetworkTableInstance.getDefault().getTable(limelightName);
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        DogLog.log("FuelDetection/LimelightName", limelightName);
    }

    @Override
    public boolean isFuelPresent()
    {
        DogLog.log("FuelDetection/Connected", table.containsKey("tv"));
        return LimelightHelpers.getTV(limelightName);
    }

    @Override
    public Angle getFuelBlobXOffsets()
    {
        return Degrees.of(-LimelightHelpers.getTX(limelightName));
    }

    @Override
    public Angle getFuelBlobYOffsets()
    {
        return Degrees.of(-LimelightHelpers.getTY(limelightName));
    }

}