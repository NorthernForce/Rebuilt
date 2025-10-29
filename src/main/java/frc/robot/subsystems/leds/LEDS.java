package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    private CANdle candle;
    private CANdleConfiguration config;
    private int length;
    private double brightness = 0.5;
    private final RGBWColor offColor = new RGBWColor(0, 0, 0, 0);

    public static enum AnimationType
    {
        None, ColorFlow, Fire, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff,
    }

    private AnimationType currentAnimation = AnimationType.None;
    private RGBWColor animationColor = new RGBWColor(255, 0, 0, 0);
    private int animationSpeed = 25;
    private AnimationDirectionValue animationDirection = AnimationDirectionValue.Forward;
    private double animationHz = 10.0;
    private int[] animationSegment =
    { 0, length - 1 };
    private double animationFireCooling = 0.4;
    private double animationFireSparking = 0.2;

    public LEDS(int CANID, int length)
    {
        candle = new CANdle(CANID);
        config = new CANdleConfiguration();
        this.length = length;
        config.LED.StripType = StripTypeValue.RGB;
        candle.getConfigurator().apply(config);
    }

    public void setColor(int red, int green, int blue)
    {
        RGBWColor newColor = new RGBWColor(red, green, blue, 0);
        candle.setControl(new SolidColor(0, length).withColor(newColor));
        currentAnimation = AnimationType.None;
    }

    public void setBrightness(double brightness)
    {
        if (brightness >= 0.0 && brightness <= 1.0)
        {
            this.brightness = brightness;
            config.LED.BrightnessScalar = brightness;
            candle.getConfigurator().apply(config);
        }
    }

    public void setAnimation(AnimationType animation)
    {
        currentAnimation = animation;
    }

    public void setAnimationColor(int red, int green, int blue)
    {
        animationColor = new RGBWColor(red, green, blue, 0);
    }

    public void setAnimationSpeed(int speed)
    {
        animationSpeed = speed;
    }

    public void setAnimationDirection(AnimationDirectionValue direction)
    {
        animationDirection = direction;
    }

    public void setAnimationHz(double hz)
    {
        if (hz <= 1000 && hz >= 2)
        {
            animationHz = hz;
        }
    }

    public void setAnimationSegment(int start, int end)
    {
        if (start >= 0 && end < length && start < end)
        {
            animationSegment[0] = start;
            animationSegment[1] = end;
        }
    }

    public void setFireAnimationParameters(double cooling, double sparking)
    {
        if (cooling >= 0.0 && cooling <= 1.0)
        {
            animationFireCooling = cooling;
        }
        if (sparking >= 0.0 && sparking <= 1.0)
        {
            animationFireSparking = sparking;
        }
    }

    public void setAnimationFireCooling(double cooling)
    {
        if (cooling >= 0.0 && cooling <= 1.0)
        {
            animationFireCooling = cooling;
        }
    }

    public void setAnimationFireSparking(double sparking)
    {
        if (sparking >= 0.0 && sparking <= 1.0)
        {
            animationFireSparking = sparking;
        }
    }

    @Override
    public void periodic()
    {
        switch (currentAnimation)
        {
        case None:
            candle.setControl(new SolidColor(animationSegment[0], animationSegment[1]).withColor(offColor));
            break;
        case ColorFlow:
        {
            ColorFlowAnimation animColorFlow = new ColorFlowAnimation(animationSegment[0], animationSegment[1])
                    .withSlot(0).withColor(animationColor).withDirection(animationDirection).withFrameRate(animationHz);
            candle.setControl(animColorFlow);
        }
            break;
        case Fire:
            FireAnimation animFire = new FireAnimation(animationSegment[0], animationSegment[1]).withSlot(0)
                    .withFrameRate(animationSpeed).withDirection(animationDirection).withCooling(animationFireCooling)
                    .withSparking(animationFireSparking);
            candle.setControl(animFire);
            break;
        case Rainbow:
            RainbowAnimation animRainbow = new RainbowAnimation(animationSegment[0], animationSegment[1]).withSlot(0)
                    .withDirection(animationDirection).withFrameRate(animationHz);
            candle.setControl(animRainbow);
            break;
        case RgbFade:
            RgbFadeAnimation animRgbFade = new RgbFadeAnimation(animationSegment[0], animationSegment[1])
                    .withFrameRate(animationHz);
            candle.setControl(animRgbFade);
            break;
        case SingleFade:
            SingleFadeAnimation animSingleFade = new SingleFadeAnimation(animationSegment[0], animationSegment[1])
                    .withColor(animationColor).withFrameRate(animationHz);
            candle.setControl(animSingleFade);
            break;
        case Strobe:
            StrobeAnimation animStrobe = new StrobeAnimation(animationSegment[0], animationSegment[1])
                    .withColor(animationColor).withFrameRate(animationHz);
            candle.setControl(animStrobe);
            break;
        case Twinkle:
            TwinkleAnimation animTwinkle = new TwinkleAnimation(animationSegment[0], animationSegment[1]).withSlot(0)
                    .withColor(animationColor).withFrameRate(animationHz);
            candle.setControl(animTwinkle);
            break;
        case TwinkleOff:
            TwinkleOffAnimation animTwinkleOff = new TwinkleOffAnimation(animationSegment[0], animationSegment[1])
                    .withSlot(0).withColor(animationColor).withFrameRate(animationHz);
            candle.setControl(animTwinkleOff);
            break;
        }
    }
}
