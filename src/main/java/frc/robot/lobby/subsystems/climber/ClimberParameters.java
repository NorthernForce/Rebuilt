package frc.robot.lobby.subsystems.climber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public record ClimberParameters(int motorID, Angle topSoftRotations, Angle bottomSoftRotations, Angle startRotations,
        Angle tolerance, boolean inverted, Pose2d upperRedPrepPose, Pose2d lowerRedPrepPose, Pose2d upperBluePrepPose,
        Pose2d lowerBluePrepPose, double dutyCyclePower, double gearRatio, Distance maxHeight) {

}
