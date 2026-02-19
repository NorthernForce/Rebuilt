package frc.robot.lobby.subsystems.spindexer.carousel;

import edu.wpi.first.units.measure.AngularVelocity;

public interface CarouselIO
{
    public static record CarouselConstants(int kMotorId, double kSpeed, double kGearRatio, boolean kInverted) {
    };

    public void startCarousel();

    public void stopCarousel();

    public AngularVelocity getSpeed();

    public void update();

    public double getPower();

    public void setPower(double power);

    public double getTargetPower();
}
