package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import java.util.stream.IntStream;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;
import frc.robot.subsystems.climber.commands.HomeCommand;

public class Climber extends SubsystemBase
{
    private final ClimberIO climber;

    public Climber(ClimberIO climber)
    {
        this.climber = climber;
    }

    public Command getHomingCommand()
    {
        return new HomeCommand(this);
    }

    public void homeDown()
    {
        climber.homeDown();
    }

    public boolean isAtBottom()
    {
        return climber.atBottom();
    }

    public boolean isAtTop()
    {
        return climber.atTop();
    }

    public Command runToPosition(ClimbLevels level)
    {
        return runToPosition(level.getLevel());
    }

    /**
     * Runs the climb sequence for the specified number of levels. Each level:
     * extend up, wait until at top, then retract down (hooks latch mechanically).
     */
    public Command runToPosition(int level)
    {
        return Commands.sequence(IntStream.range(0, level).mapToObj(i -> Commands.sequence(
                // Extend elevator to top
                runOnce(() -> climber.runUp()),
                // Wait until elevator reaches top
                Commands.waitUntil(this::isAtTop),
                // Retract elevator to bottom (pulls robot up, hooks latch)
                getHomingCommand())).toArray(Command[]::new));
    }

    public void stopMotor()
    {
        climber.stopMotor();
    }

    @Override
    public void simulationPeriodic()
    {
        climber.updateSimulation();
    }

    @Override
    public void periodic()
    {
        DogLog.log("Elevator/ElevatorRotations", climber.getRotations());
        DogLog.log("Elevator/AtTop", climber.atTop());
        DogLog.log("Elevator/AtBottom", climber.atBottom());
    }
}
