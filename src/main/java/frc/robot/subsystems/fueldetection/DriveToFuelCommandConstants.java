package frc.robot.subsystems.fueldetection;

import edu.wpi.first.units.measure.Angle;

public record DriveToFuelCommandConstants(double turnSpeedMultiplier, double forwardSpeedMultiplier, Angle zeroAngle,
        Angle zeroXAngle, double forwardExponent, double turnExponent) {

}