package frc.robot.lobby;

import java.util.Map;

public class TargetingData
{
    public static final Map<Double, Double> HOOD_DATA = Map.ofEntries(Map.entry(1.80, 21.0), Map.entry(2.0, 21.0),
            Map.entry(2.20, 21.0), Map.entry(2.40, 21.0), Map.entry(2.60, 21.0), Map.entry(2.80, 21.0),
            Map.entry(3.00, 21.0), Map.entry(3.20, 21.0), Map.entry(3.40, 23.0), Map.entry(3.60, 25.0),
            Map.entry(3.80, 27.0), Map.entry(4.00, 28.0), Map.entry(4.25, 31.0), Map.entry(4.44, 31.0),
            Map.entry(4.72, 32.5), Map.entry(5.05, 34.0), Map.entry(5.34, 34.0));

    public static final Map<Double, Double> SHOOTER_DATA = Map.ofEntries(Map.entry(1.80, 45.0), Map.entry(2.0, 46.0),
            Map.entry(2.20, 47.5), Map.entry(2.40, 48.5), Map.entry(2.60, 50.5), Map.entry(2.80, 52.1),
            Map.entry(3.00, 53.15), Map.entry(3.20, 54.2), Map.entry(3.40, 55.25), Map.entry(3.60, 56.5),
            Map.entry(3.80, 57.25), Map.entry(4.00, 57.5), Map.entry(4.23, 57.9), Map.entry(4.44, 58.5),
            Map.entry(4.72, 58.7), Map.entry(5.05, 59.0), Map.entry(5.34, 61.0));
}
