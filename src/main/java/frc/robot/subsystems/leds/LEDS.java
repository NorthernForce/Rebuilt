package frc.robot.subsystems.leds;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    LedsIO io;

    enum GameState
    {
        DISABLED, AUTONOMOUS, TELEOP, TRANSITION, ALLIANCE_SHIFT1, ALLIANCE_SHIFT2, ALLIANCE_SHIFT3, ALLIANCE_SHIFT4,
        END_GAME, TEST
    }

    private boolean shiftChangeSoon = false;

    private GameState gameState;

    private boolean hubActive;

    private double matchTime = 0.0;

    private Optional<Alliance> alliance;

    public LEDS(LedsIO ledIO, int id, int length, double brightness)
    {
        io = ledIO;

        alliance = DriverStation.getAlliance();
    }

    public void setColor(int red, int green, int blue)
    {
        io.setColor(red, green, blue);
    }

    public void setBrightness(double brightness)
    {
        io.setBrightness(brightness);
    }

    public void setLength(int length)
    {
        io.setLength(length);
    }

    public void rainbowAnimation()
    {
        io.rainbowAnimation();
    }

    public void setGameState()
    {
        matchTime = DriverStation.getMatchTime();
        if (DriverStation.isDisabled())
        {
            gameState = GameState.DISABLED;
        } else if (DriverStation.isAutonomous())
        {
            gameState = GameState.AUTONOMOUS;
        } else if (DriverStation.isTest())
        {
            gameState = GameState.TEST;
        } else
        {
            if (matchTime > 130)
            {
                gameState = GameState.TRANSITION;
                if (matchTime <= 135)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 105)
            {
                gameState = GameState.ALLIANCE_SHIFT1;
                if (matchTime <= 110)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 80)
            {
                gameState = GameState.ALLIANCE_SHIFT2;
                if (matchTime <= 85)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 55)
            {
                gameState = GameState.ALLIANCE_SHIFT3;
                if (matchTime <= 60)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 30)
            {
                gameState = GameState.ALLIANCE_SHIFT4;
                if (matchTime <= 35)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else
            {
                gameState = GameState.END_GAME;
                if (matchTime <= 5)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            }
        }

        boolean redInactiveFirst = switch (DriverStation.getGameSpecificMessage().charAt(0))
        {
        case 'R' -> true;
        case 'B' -> false;
        default -> false;
        };

        boolean firstShiftActive = switch (alliance.get())
        {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
        default -> false;
        };

        hubActive = switch (gameState)
        {
        case AUTONOMOUS, TRANSITION, END_GAME -> true;
        case ALLIANCE_SHIFT1 -> firstShiftActive;
        case ALLIANCE_SHIFT2 -> !firstShiftActive;
        case ALLIANCE_SHIFT3 -> firstShiftActive;
        case ALLIANCE_SHIFT4 -> !firstShiftActive;
        default -> false;
        };
    }

    @Override
    public void periodic()
    {
        setGameState();

        // If the robot is disabled, show the rainbow animation
        if (DriverStation.isDisabled())
        {
            rainbowAnimation();
            return;
        } else if (hubActive)
        {
            if (shiftChangeSoon)
            {
                io.blinkAnimation(0, 255, 0);
            } else
            {
                setColor(0, 255, 0);
            }
        } else
        {
            if (shiftChangeSoon)
            {
                io.blinkAnimation(255, 0, 0);
            } else
            {
                setColor(255, 0, 0);
            }
        }
    }

}
