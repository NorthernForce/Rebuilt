package frc.robot.lobby.subsystems.spindexer.carousel;

import edu.wpi.first.units.measure.AngularVelocity;

public interface CarouselIO
{
    public static record CarouselConstants(int kMotorId, AngularVelocity kSpeed, double kGearRatio, double kV,
            double kA, double kP, double kI, double kD, AngularVelocity kErrorTolerance, boolean kInverted) {
    };

    public void startCarousel();

    public void stopCarousel();

    public AngularVelocity getSpeed();

    public void update();
}
