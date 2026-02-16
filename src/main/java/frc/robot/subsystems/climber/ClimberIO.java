package frc.robot.subsystems.climber;

import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public interface ClimberIO
{
    public void runUp();

    public void homeDown();

    public ClimbLevels getLevel();

    public boolean atBottom();

    public boolean atTop();

    public void stopMotor();

    public double getRotations();

    public default void updateSimulation()
    {
    }
}
