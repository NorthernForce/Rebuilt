package frc.robot.lobby;

import java.util.Map;

import edu.wpi.first.units.measure.Time;

public class TargetingData
{
    public static final Map<Double, Double> HOOD_DATA = Map.ofEntries(Map.entry(1.80, 21.0), Map.entry(2.0, 21.0),
            Map.entry(2.20, 21.0), Map.entry(2.40, 21.0), Map.entry(2.60, 21.0), Map.entry(2.80, 21.0),
            Map.entry(3.00, 21.0), Map.entry(3.20, 21.0), Map.entry(3.40, 23.0), Map.entry(3.60, 25.0),
            Map.entry(3.80, 27.0), Map.entry(4.00, 28.0), Map.entry(4.25, 31.0), Map.entry(4.44, 31.0),
            Map.entry(4.72, 33.0), Map.entry(5.05, 35.0));

    public static final Map<Double, Double> SHOOTER_DATA = Map.ofEntries(Map.entry(1.80, 44.0), Map.entry(2.0, 46.0),
            Map.entry(2.20, 47.0), Map.entry(2.40, 49.0), Map.entry(2.60, 51.5), Map.entry(2.80, 53.1),
            Map.entry(3.00, 55.0), Map.entry(3.20, 57.5), Map.entry(3.40, 58.5), Map.entry(3.60, 58.5),
            Map.entry(3.80, 57.5), Map.entry(4.00, 58.0), Map.entry(4.25, 58.0), Map.entry(4.44, 59.5),
            Map.entry(4.72, 60.5), Map.entry(5.05, 65.0));
//     public static final Map<Double, Double> TOF_DATA = Map.of(0.0, 0.41353, 6.0, 1.99294);

    public static final Map<Double, Double> TOF_DATA = Map.ofEntries(Map.entry(1.8, 4.66 - 3.82),
            Map.entry(2.0, 2.92 - 1.96), Map.entry(2.4, 2.68 - 1.65), Map.entry(2.6, 3.06 - 1.97),
            Map.entry(2.8, 18.48 - 17.28), Map.entry(3.0, 4.72 - 3.49), Map.entry(3.2, 21.38 - 20.03),
            Map.entry(3.4, 3.85 - 2.61), Map.entry(3.6, 33.07 - 34.38), Map.entry(4.0, 20.45 - 19.19),
            Map.entry(6.0, 1.99294));
}
