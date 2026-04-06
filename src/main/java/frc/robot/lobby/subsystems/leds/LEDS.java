package frc.robot.lobby.subsystems.leds;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    LedsIO io;
    private boolean connected = false;
    private Timer timer;

    enum GameState
    {
        DISCONNECTED, DISABLED, AUTONOMOUS, TELEOP, TRANSITION, ALLIANCE_SHIFT1, ALLIANCE_SHIFT2, ALLIANCE_SHIFT3,
        ALLIANCE_SHIFT4, END_GAME, TEST
    }

    private boolean shiftChangeSoon = false;

    private GameState gameState;

    private boolean hubActive;

    private double matchTime = 0.0;

    private Optional<Alliance> alliance;

    public LEDS(LedsIO ledIO)
    {
        io = ledIO;

        alliance = DriverStation.getAlliance();
        timer = new Timer();
    }

    public void setColor(int red, int green, int blue)
    {
        io.setColor(red, green, blue);
    }

    public void setColor(Color color)
    {
        io.setColor((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }

    public void movingColor(int red, int green, int blue)
    {
        io.movingColor(red, green, blue);
    }

    public void movingColor(Color color)
    {
        io.movingColor((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }

    public void blinkAnimation(int red, int green, int blue)
    {
        io.blinkAnimation(red, green, blue);
    }

    public void blinkAnimation(int red, int green, int blue, double frameRate)
    {
        io.blinkAnimation(red, green, blue, frameRate);
    }

    public void blinkAnimation(Color color)
    {
        io.blinkAnimation((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    }

    public void blinkAnimation(Color color, double frameRate)
    {
        io.blinkAnimation((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), frameRate);
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
        if (!DriverStation.isDSAttached())
        {
            gameState = GameState.DISCONNECTED;
        } else if (DriverStation.isDisabled())
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
                if (matchTime <= 140)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 105)
            {
                gameState = GameState.ALLIANCE_SHIFT1;
                if (matchTime <= 115)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 80)
            {
                gameState = GameState.ALLIANCE_SHIFT2;
                if (matchTime <= 90)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 55)
            {
                gameState = GameState.ALLIANCE_SHIFT3;
                if (matchTime <= 65)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else if (matchTime > 30)
            {
                gameState = GameState.ALLIANCE_SHIFT4;
                if (matchTime <= 40)
                {
                    shiftChangeSoon = true;
                } else
                {
                    shiftChangeSoon = false;
                }
            } else
            {
                gameState = GameState.END_GAME;
                if (matchTime <= 10)
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

        if (gameState == GameState.DISCONNECTED)
        {
            if (connected)
            {
                blinkAnimation(kOrange, 4);
            } else
            {
                movingColor(kPink);
            }
        } else
        {
            if (!connected)
            {
                connected = true;
                timer.restart();
                blinkAnimation(kPink, 2);
            }
            if (gameState == GameState.DISABLED && timer.hasElapsed(2.0))
            {
                if (alliance.orElse(Alliance.Blue) == Alliance.Blue)
                {
                    setColor(kBlue);
                } else
                {
                    setColor(kRed);
                }
            } else if (gameState == GameState.AUTONOMOUS)
            {
                setColor(kGreen);
            } else if (hubActive)
            {
                if (shiftChangeSoon)
                {
                    blinkAnimation(kGreen);
                } else
                {
                    setColor(kGreen);
                }
            } else
            {
                if (shiftChangeSoon)
                {
                    blinkAnimation(kPink);
                } else
                {
                    setColor(kPink);
                }
            }
        }
    }

    public static final Color kPink = new Color(239, 48, 125);
    public static final Color kGreen = new Color(65, 230, 53);
    public static final Color kOrange = new Color(255, 170, 0);
    public static final Color kRed = new Color(255, 0, 0);
    public static final Color kBlue = new Color(0, 0, 255);
}