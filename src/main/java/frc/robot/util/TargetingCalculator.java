package frc.robot.util;

public interface TargetingCalculator
{
    public double getValueForDistance(double distance);

    public void addData(double distance, double value);
}
