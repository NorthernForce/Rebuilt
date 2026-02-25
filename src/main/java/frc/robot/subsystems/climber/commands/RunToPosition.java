package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lobby.LobbyContainer;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.apriltagvision.commands.DriveToPoseWithVision;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.RepeatCommand;

public class RunToPosition extends SequentialCommandGroup
{
    public RunToPosition(LobbyContainer container, ClimbLevels level)
    {
        addCommands(
                Commands.parallel(container.driveToPreClimbPosition(),
                        Commands.sequence(container.getIntake().getRunToStowAngleCommand(),
                                container.getClimber().extendHooks(), container.getClimber().runUp())),
                container.driveToClimbPost(), container.getClimber().runToPosition(level)

        );
    }
}
