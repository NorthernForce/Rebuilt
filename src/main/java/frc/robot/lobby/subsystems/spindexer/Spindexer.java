package frc.robot.lobby.subsystems.spindexer;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lobby.subsystems.spindexer.carousel.CarouselIO;
import frc.robot.lobby.subsystems.spindexer.flicker.FlickerIO;

public class Spindexer extends SubsystemBase
{
    protected final CarouselIO m_carousel;
    protected final FlickerIO m_flicker;
    protected final Time deJamTime;

    public Spindexer(CarouselIO carousel, FlickerIO flicker, SpindexerParameters constants)
    {
        m_carousel = carousel;
        m_flicker = flicker;
        deJamTime = constants.deJamTime();
    }

    public CarouselIO getCarousel()
    {
        return m_carousel;
    }

    public FlickerIO getFlicker()
    {
        return m_flicker;
    }

    public boolean getJammed()
    {
        return m_flicker.getJammed() || m_carousel.getJammed();
    }

    public Command runBackwards()
    {
        return Commands.deadline(Commands.waitTime(deJamTime), Commands.run(() ->
        {
            m_carousel.dejam();
            m_flicker.dejam();
        }, this)).andThen(Commands.runOnce(() ->
        {
            m_carousel.stopCarousel();
            m_flicker.stopFlicker();
            m_carousel.resetJamDetection();
            m_flicker.resetJamDetection();
        }, this));

    }

    @Override
    public void periodic()
    {
        m_carousel.update();
    }

    public record SpindexerParameters(Time deJamTime) {

    }
}
