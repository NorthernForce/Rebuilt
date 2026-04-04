package frc.robot.lobby.subsystems.leds;

public interface LedsIO
{

    public default void setColor(int red, int green, int blue)
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

}