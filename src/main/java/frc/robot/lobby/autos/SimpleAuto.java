package frc.robot.lobby.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lobby.LobbyConstants;
import frc.robot.lobby.LobbyContainer;
import frc.robot.lobby.subsystems.spindexer.commands.RunSpindexer;
import frc.robot.lobby.subsystems.turret.commands.PrepTurretCommand;

public class SimpleAuto extends SequentialCommandGroup
{
    public SimpleAuto(LobbyContainer container, Pose2d startingPose)
    {
        addCommands(Commands.runOnce(() -> container.getDrive().resetPose(startingPose)), Commands
                .waitUntil(() -> container.getTurret().getSuzie().isAtTargetAngle()
                        && container.getTurret().getShooter().isAtTargetSpeed())
                .andThen(new RunSpindexer(container.getSpindexer(), LobbyConstants.SpindexerConstants.kDeJamTime,
                        LobbyConstants.SpindexerConstants.kPostDeJamTime))
                .alongWith(new PrepTurretCommand(container)));
    }
}
