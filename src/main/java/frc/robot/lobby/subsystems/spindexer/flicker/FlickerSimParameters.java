package frc.robot.lobby.subsystems.spindexer.flicker;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public record FlickerSimParameters(int motorId, double rampSpeed, double gearRatio, double kV, double kP, double kI,
        double kD, double errorTolerance, double simMaxRpm, double simMoi, Current jamCurrentThreshold, Time jamTimeout,
        double dejamSpeed) {

}
