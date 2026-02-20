package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public class TrigHoodTargetingCalculator implements TargetingCalculator
{
    @Override
    public double getValueForDistance(double distance)
    {
        // Get the correct hub height based on alliance
        double hubHeight = getHubHeight();
        return Math.PI / 2.0 - Math.atan(Math.abs(hubHeight / distance));
    }

    private double getHubHeight()
    {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red)
        {
            return FieldConstants.kRedHubPosition.getZ();
        }
        return FieldConstants.kBlueHubPosition.getZ();
    }

    @Override
    public void addData(double distance, double value)
    {
    }
}
