package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants
{
    // Field dimensions (2020 Infinite Recharge / 2022 Rapid React style)
    public static final double kFieldLengthMeters = 16.4592; // 54 feet
    public static final double kFieldWidthMeters = 8.2296; // 27 feet

    // Hub positions (center of field, raised)
    public static final Translation3d kBlueHubPosition = new Translation3d(Meters.of(kFieldLengthMeters / 4.0), // Blue
                                                                                                                // side
            Meters.of(kFieldWidthMeters / 2.0), // Center width
            Meters.of(2.64)); // Hub height

    public static final Translation3d kRedHubPosition = new Translation3d(Meters.of(kFieldLengthMeters * 3.0 / 4.0), // Red
                                                                                                                     // side
            Meters.of(kFieldWidthMeters / 2.0), // Center width
            Meters.of(2.64)); // Hub height

    // Legacy - used by existing code, will be set based on alliance
    public static Translation3d HubPosition = kBlueHubPosition;

    // Trench centers (4 total - 2 on each side, mirrored)
    // Blue side trenches
    public static final Translation2d kBlueTrench1 = new Translation2d((651.22 / 2 - 143.50) * 0.0254,
            ((317.69 + (317.69 - 49.84)) / 2.0) * 0.0254);
    public static final Translation2d kBlueTrench2 = new Translation2d((651.22 / 2 - 143.50) * 0.0254,
            kFieldWidthMeters - ((317.69 + (317.69 - 49.84)) / 2.0) * 0.0254); // Mirrored across center Y

    // Red side trenches (mirrored across center X)
    public static final Translation2d kRedTrench1 = new Translation2d(
            kFieldLengthMeters - (651.22 / 2 - 143.50) * 0.0254, ((317.69 + (317.69 - 49.84)) / 2.0) * 0.0254);
    public static final Translation2d kRedTrench2 = new Translation2d(
            kFieldLengthMeters - (651.22 / 2 - 143.50) * 0.0254,
            kFieldWidthMeters - ((317.69 + (317.69 - 49.84)) / 2.0) * 0.0254); // Mirrored across center Y

    // Legacy names for compatibility
    public static final Translation2d kCenterBlueTrench = kBlueTrench1;
    public static final Translation2d kCenterRedTrench = kRedTrench1;

    // Alliance station positions (X coordinates)
    public static final double kBlueAllianceStationX = 0.0;
    public static final double kRedAllianceStationX = kFieldLengthMeters;

    // Alliance station corners for passing (when out of shooting range)
    public static final Translation2d kBluePassingTarget = new Translation2d(Meters.of(1.0), // Near blue alliance wall
            Meters.of(kFieldWidthMeters - 1.0)); // Left corner (from driver perspective)

    public static final Translation2d kRedPassingTarget = new Translation2d(Meters.of(kFieldLengthMeters - 1.0), // Near
                                                                                                                 // red
                                                                                                                 // alliance
                                                                                                                 // wall
            Meters.of(1.0)); // Left corner (from driver perspective)

    // Maximum shooting distance (meters)
    public static final double kMaxShootingDistance = 8.0;
}
