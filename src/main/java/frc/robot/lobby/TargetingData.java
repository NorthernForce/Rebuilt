package frc.robot.lobby;

import java.util.Map;

public class TargetingData
{
    public static final Map<Double, Double> HOOD_DATA = Map.ofEntries(Map.entry(1.80, 21.0), Map.entry(2.0, 21.0),
            Map.entry(2.20, 21.0), Map.entry(2.40, 21.0), Map.entry(2.60, 21.0), Map.entry(2.80, 21.0),
            Map.entry(3.00, 21.0), Map.entry(3.20, 21.0), Map.entry(3.40, 23.0), Map.entry(3.60, 25.0),
            Map.entry(3.80, 27.0), Map.entry(4.00, 28.0), Map.entry(4.25, 31.0), Map.entry(4.80, 34.0),
            Map.entry(5.6, 34.0));

    public static final Map<Double, Double> SHOOTER_DATA = Map.ofEntries(Map.entry(1.80, 45.0), Map.entry(2.0, 46.0),
            Map.entry(2.20, 47.0), Map.entry(2.40, 48.0), Map.entry(2.60, 45.0), Map.entry(2.80, 50.5),
            Map.entry(3.00, 51.5), Map.entry(3.20, 53.5), Map.entry(3.40, 54.5), Map.entry(3.60, 56.25),
            Map.entry(3.80, 59.0), Map.entry(4.00, 58.0), Map.entry(4.25, 57.5), Map.entry(4.80, 58.0),
            Map.entry(5.6, 63.0));
}