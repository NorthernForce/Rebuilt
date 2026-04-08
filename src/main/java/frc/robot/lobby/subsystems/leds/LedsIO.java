package frc.robot.lobby.subsystems.leds;

import com.ctre.phoenix6.signals.AnimationDirectionValue;

import edu.wpi.first.wpilibj.util.Color;

public interface LedsIO
{
    public static record LedsConstants(int id, int length, double brightness,
            AnimationDirectionValue animationDirection) {
    }

    public default void setColor(Color color)
    {
    }

    public default void movingColor(Color color)
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

    public default void blinkAnimation(Color color)
    {
    }

    public default void blinkAnimation(Color color, double frameRate)
    {
    }

    public default void clear()
    {
    }

}