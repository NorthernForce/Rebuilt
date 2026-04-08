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

import edu.wpi.first.wpilibj.util.Color;

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
        config.LED.StripType = StripTypeValue.GRB;
        length = constants.length();
        config.LED.BrightnessScalar = constants.brightness();
        candle.getConfigurator().apply(config);

        animationDirection = constants.animationDirection();
        candle.clearAllAnimations();
        movingColor(Color.kMagenta);
    }

    @Override
    public void setColor(Color color)
    {
        RGBWColor newColor = new RGBWColor(color);
        candle.setControl(new SolidColor(0, length).withColor(newColor));
    }

    @Override
    public void movingColor(Color color)
    {
        RGBWColor newColor = new RGBWColor(color);
        candle.setControl(new LarsonAnimation(0, length).withBounceMode(LarsonBounceValue.Front).withFrameRate(4)
                .withSize(3).withColor(newColor));
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
    public void blinkAnimation(Color color)
    {
        RGBWColor newColor = new RGBWColor(color);
        candle.setControl(new StrobeAnimation(0, length).withColor(newColor).withFrameRate(4));
    }

    @Override
    public void blinkAnimation(Color color, double frameRate)
    {
        RGBWColor newColor = new RGBWColor(color);
        candle.setControl(new StrobeAnimation(0, length).withColor(newColor).withFrameRate(frameRate));
    }

    @Override
    public void clear()
    {
        candle.clearAllAnimations();
    }
}