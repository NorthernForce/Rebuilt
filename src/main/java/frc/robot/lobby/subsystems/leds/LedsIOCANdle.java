package frc.robot.lobby.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

public class LedsIOCANdle implements LedsIO
{
    private CANdle candle;
    private CANdleConfiguration config;

    private int length;

    private AnimationDirectionValue animationDirection;

    public LedsIOCANdle(LedsConstants constants)
    {
        candle = new CANdle(constants.id());
        config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.RGB;
        length = constants.length();
        config.LED.BrightnessScalar = constants.brightness();
        candle.getConfigurator().apply(config);

        animationDirection = constants.animationDirection();
    }

    @Override
    public void setColor(int red, int green, int blue)
    {
        RGBWColor color = new RGBWColor(red, green, blue, 0);
        candle.setControl(new SolidColor(0, length).withColor(color));
    }

    @Override
    public void movingColor(int red, int green, int blue)
    {
        RGBWColor color = new RGBWColor(red, green, blue, 0);
        candle.setControl(new LarsonAnimation(0, length).withBounceMode(LarsonBounceValue.Back).withFrameRate(4)
                .withSize(3).withColor(color));
    }

    @Override
    public void setBrightness(double brightness)
    {
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

    @Override
    public void blinkAnimation(int red, int green, int blue)
    {
        RGBWColor newColor = new RGBWColor(red, green, blue, 0);
        StrobeAnimation animation = new StrobeAnimation(0, length).withSlot(0).withColor(newColor).withFrameRate(4);
        candle.setControl(animation);
    }

    @Override
    public void blinkAnimation(int red, int green, int blue, double frameRate)
    {
        RGBWColor newColor = new RGBWColor(red, green, blue, 0);
        StrobeAnimation animation = new StrobeAnimation(0, length).withSlot(0).withColor(newColor)
                .withFrameRate(frameRate);
        candle.setControl(animation);
    }
}