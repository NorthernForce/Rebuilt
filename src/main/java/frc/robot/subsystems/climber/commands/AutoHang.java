package frc.robot.subsystems.climber.commands;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lobby.LobbyContainer;
import frc.robot.lobby.subsystems.intake.Intake;
import frc.robot.subsystems.climber.Climber;

public class AutoHang extends SequentialCommandGroup
{
    public AutoHang(LobbyContainer container)
    {
        Climber climber = container.getClimber();
        Intake intake = container.getIntake();
        addCommands(
                Commands.runOnce(() -> DogLog.log("AutoHang/Step", "DriveToPreClimb+Stow")),
                Commands.parallel(container.driveToPreClimbPosition(), intake.getRunToStowAngleCommand()),
                Commands.runOnce(() -> DogLog.log("AutoHang/Step", "ExtendHooks")),
                climber.extendHooks(),
                Commands.runOnce(() -> DogLog.log("AutoHang/Step", "RunUp")),
                climber.runUp(),
                Commands.runOnce(() -> DogLog.log("AutoHang/Step", "DriveToClimbPost")),
                container.driveToClimbPost(),
                Commands.runOnce(() -> DogLog.log("AutoHang/Step", "HomeDown")),
                Commands.run(() -> climber.homeDown(), climber),
                Commands.waitUntil(() -> !DriverStation.isAutonomous()),
                Commands.runOnce(() -> DogLog.log("AutoHang/Step", "RunUp2")),
                climber.runUp(),
                container.driveToPreClimbPosition());
    }
}
