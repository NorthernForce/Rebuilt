package frc.robot.lobby.subsystems.leds;

import com.ctre.phoenix6.signals.AnimationDirectionValue;

public interface LedsIO
{
    public static record LedsConstants(int id, int length, double brightness,
            AnimationDirectionValue animationDirection) {
    }

    public default void setColor(int red, int green, int blue)
    {
    }

    public default void movingColor(int red, int green, int blue)
    {
    }

    public default void setBrightness(double brightness)
    {
    }

    public default void setLength(int length)
    {
    }

    public default void rainbowAnimation()
    {
    }

    public default void blinkAnimation(int red, int green, int blue)
    {
    }

    public default void blinkAnimation(int red, int green, int blue, double frameRate)
    {
    }

}