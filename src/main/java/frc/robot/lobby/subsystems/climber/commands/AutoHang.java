package frc.robot.lobby.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lobby.LobbyContainer;
import frc.robot.lobby.subsystems.climber.Climber;

import frc.robot.lobby.subsystems.intake.Intake;

public class AutoHang extends SequentialCommandGroup
{
    public AutoHang(LobbyContainer container)
    {
        Climber climber = container.getClimber();
        Intake intake = container.getIntake();
        addCommands(container.driveToPreClimbPosition().alongWith(climber.runUp())
                .alongWith(intake.getRunToStowAngleCommand()).andThen(container.driveToClimbPost())
                .andThen(climber.runDown()));
    }
}
