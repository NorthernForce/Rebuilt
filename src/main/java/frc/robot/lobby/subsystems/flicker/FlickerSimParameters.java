package frc.robot.lobby.subsystems.flicker;

public record FlickerSimParameters(int motorId, double rampSpeed, double gearRatio, double kV, double kP, double kI,
        double kD, double simMaxRpm, double simMoi) {

}
