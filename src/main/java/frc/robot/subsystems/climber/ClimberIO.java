package frc.robot.subsystems.climber;

import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public interface ClimberIO {
    public void runToPosition(ClimbLevels level);

    public void runToPosition(int level);

    public void homeDown();

    public ClimbLevels getLevel();

    public boolean atBottom();

    public void stopMotor();
}
