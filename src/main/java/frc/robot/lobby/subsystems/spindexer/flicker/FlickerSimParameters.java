package frc.robot.lobby.subsystems.spindexer.flicker;

public record FlickerSimParameters(int motorId, double rampSpeed, double gearRatio, double kV, double kP, double kI,
        double kD, double errorTolerance, double simMaxRpm, double simMoi) {

}
