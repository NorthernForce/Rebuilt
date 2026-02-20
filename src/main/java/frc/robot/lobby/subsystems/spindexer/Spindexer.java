package frc.robot.lobby.subsystems.spindexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIO;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIO;

public class Spindexer extends SubsystemBase
{
    protected final CarouselIO m_carousel;
    protected final FlickerIO m_flicker;

    public Spindexer(CarouselIO carousel, FlickerIO flicker)
    {
        m_carousel = carousel;
        m_flicker = flicker;
    }

    public CarouselIO getCarousel()
    {
        return m_carousel;
    }

    public FlickerIO getFlicker()
    {
        return m_flicker;
    }
}
