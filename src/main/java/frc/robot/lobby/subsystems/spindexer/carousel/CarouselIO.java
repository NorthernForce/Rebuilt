package frc.robot.lobby.subsystems.spindexer.carousel;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

public interface CarouselIO
{
    public static record CarouselConstants(int kMotorId, double kSpeed, double kGearRatio, boolean kInverted,
            Current kJamCurrentThreshold, Time kJamTimeout, double dejamSpeed) {
    };

    public void startCarousel();

    public void stopCarousel();

    public AngularVelocity getSpeed();

    public void update();

    public double getPower();

    public void setPower(double power);

    public double getTargetPower();

    public boolean getJammed();

    public void dejam();

    public void resetJamDetection();
}
