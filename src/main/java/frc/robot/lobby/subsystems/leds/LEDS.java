package frc.robot.lobby.subsystems.leds;

import java.util.Optional;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDS extends SubsystemBase
{
    LedsIO io;
    private boolean connected = false;

    enum GameState
    {
        DISCONNECTED, DISABLED, AUTONOMOUS, TELEOP, TRANSITION, ALLIANCE_SHIFT1, ALLIANCE_SHIFT2, ALLIANCE_SHIFT3,
        ALLIANCE_SHIFT4, END_GAME, TEST
    }

    private boolean shiftChangeSoon = false;

    private GameState gameState;

    private boolean hubActive;

    private double matchTime = 0.0;

    private Supplier<Optional<Alliance>> alliance;

    private boolean shooterPrepped = false;

    private double lastTime = DriverStation.getMatchTime();

    private boolean timeDecreasing = false;

    public LEDS(LedsIO ledIO)
    {
        io = ledIO;

        alliance = () -> DriverStation.getAlliance();
    }

    public void setColor(Color color)
    {
        if (!io.getAnimationName().equals("SolidColor"))
        {
            io.clear();
            io.setColor(color);
        }
    }

    public void movingColor(Color color)
    {
        if (!io.getAnimationName().equals("LarsonAnimation"))
        {
            io.clear();
            io.movingColor(color);
        }
    }

    public void blinkAnimation(Color color)
    {
        if (!io.getAnimationName().equals("StrobeAnimation"))
        {
            io.clear();
            io.blinkAnimation(color);
        }
    }

    public void blinkAnimation(Color color, double frameRate)
    {
        if (!io.getAnimationName().equals("StrobeAnimation"))
        {
            io.clear();
            io.blinkAnimation(color, frameRate);
        }
    }

    public void setBrightness(double brightness)
    {
        io.setBrightness(brightness);
    }

    public void setLength(int length)
    {
        io.setLength(length);
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

        boolean redInactiveFirst = false;

        if (DriverStation.getGameSpecificMessage().indexOf('R') == 0)
        {
            redInactiveFirst = true;
        }

        boolean firstShiftActive = switch (alliance.get().orElse(Alliance.Blue))
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
        DogLog.log("Leds/ShooterPrepped", shooterPrepped);
        DogLog.log("Leds/MatchType", DriverStation.getMatchType().toString());
        DogLog.log("Leds/AnimationName", io.getAnimationName());
        DogLog.log("Leds/MatchTime", DriverStation.getMatchTime());
        DogLog.log("Leds/LastMatchTime", lastTime);
        DogLog.log("Leds/IsPractice", isPractice());

        if (gameState.equals(GameState.DISCONNECTED))
        {
            if (connected)
            {
                blinkAnimation(Color.kOrange);
            } else
            {
                movingColor(Color.kMagenta);
            }
        } else if (DriverStation.isFMSAttached() || isPractice())
        {
            if (!connected)
            {
                connected = true;
            }
            if (gameState.equals(GameState.DISABLED))
            {
                if (alliance.get().isPresent())
                {
                    if (alliance.get().orElse(Alliance.Blue) == Alliance.Blue)
                    {
                        setColor(Color.kBlue);
                    } else
                    {
                        setColor(Color.kRed);
                    }
                } else
                {
                    blinkAnimation(Color.kYellow);
                }
            } else if (gameState.equals(GameState.AUTONOMOUS) || gameState.equals(GameState.TRANSITION))
            {
                if (shiftChangeSoon)
                {
                    blinkAnimation(Color.kGreen);
                } else
                {
                    setColor(Color.kGreen);
                }
            } else if (gameState.equals(GameState.END_GAME))
            {
                blinkAnimation(Color.kGreen, 2);
            } else if (hubActive)
            {
                if (shiftChangeSoon)
                {
                    blinkAnimation(Color.kGreen);
                } else
                {
                    setColor(Color.kGreen);
                }
            } else
            {
                if (shiftChangeSoon)
                {
                    blinkAnimation(Color.kMagenta);
                } else
                {
                    setColor(Color.kMagenta);
                }
            }
        } else
        {
            if (shooterPrepped)
            {
                blinkAnimation(Color.kMagenta);
            } else
            {
                setColor(Color.kMagenta);
            }
        }
    }

    public void setShooterPrepped(boolean shooterPrepped)
    {
        this.shooterPrepped = shooterPrepped;
    }

    public boolean isPractice()
    {
        if (matchTime != lastTime)
        {
            timeDecreasing = matchTime < lastTime;
            lastTime = matchTime;
            return timeDecreasing;
        } else
        {
            return timeDecreasing;
        }
    }

    public String getGameStateString()
    {
        return gameState.toString();
    }
}