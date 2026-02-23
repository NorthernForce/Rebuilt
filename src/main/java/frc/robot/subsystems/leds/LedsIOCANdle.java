package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import frc.robot.lobby.LobbyConstants;

public class LedsIOCANdle implements LedsIO
{
    private CANdle candle;
    private CANdleConfiguration config;

    private int length;
    private double brightness;
    private AnimationDirectionValue animationDirection;

    public LedsIOCANdle()
    {
        candle = new CANdle(LobbyConstants.LEDConstants.kCANdleId);
        config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        length = LobbyConstants.LEDConstants.kLength;
        brightness = LobbyConstants.LEDConstants.kBrightness;
        config.LED.BrightnessScalar = brightness;
        candle.getConfigurator().apply(config);

        animationDirection = LobbyConstants.LEDConstants.kAnimationDirection;
    }

    @Override
    public void setColor(int red, int green, int blue)
    {
        RGBWColor color = new RGBWColor(red, green, blue, 0);
        candle.setControl(new SolidColor(0, length).withColor(color));
    }

    @Override
    public void setBrightness(double brightness)
    {
        this.brightness = brightness;
        config.LED.BrightnessScalar = brightness;
        candle.getConfigurator().apply(config);
    }

    @Override
    public void setLength(int length)
    {
        this.length = length;
    }

    @Override
    public void rainbowAnimation()
    {
        RainbowAnimation animation = new RainbowAnimation(0, length).withSlot(0).withDirection(animationDirection);
        candle.setControl(animation);
    }
}