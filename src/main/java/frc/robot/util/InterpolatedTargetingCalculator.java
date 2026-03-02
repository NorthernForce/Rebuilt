package frc.robot.util;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatedTargetingCalculator implements TargetingCalculator
{
    private InterpolatingDoubleTreeMap treeMap;

    public InterpolatedTargetingCalculator(Map<Double, Double> data)
    {
        treeMap = new InterpolatingDoubleTreeMap();
        data.forEach((distance, value) -> treeMap.put(distance, value));
    }

    @Override
    public double getValueForDistance(double distance)
    {
        distance = ((int) (distance * 100)) / 100.0;
        try
        {
            return treeMap.get(distance);
        } catch (Exception e)
        {
            e.printStackTrace();
        }
        return 0;
    }
}