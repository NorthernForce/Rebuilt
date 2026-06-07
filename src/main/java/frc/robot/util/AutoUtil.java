package frc.robot.util;

import java.util.function.Function;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lobby.subsystems.CommandSwerveDrivetrain;
import frc.robot.lobby.subsystems.nfrdashboard.Dashboard;

public class AutoUtil
{
    private final AutoFactory factory;
    private final SendableChooser<Command> chooser;
    private final Dashboard dashboard;

    public AutoUtil(CommandSwerveDrivetrain drive, PIDController xPid, PIDController yPid, PIDController rPid,
            Dashboard dashboard)
    {
        factory = new AutoFactory(() -> drive.getPose(), pose -> drive.resetTranslation(pose.getTranslation()),
                (SwerveSample sample) ->
                {
                    var pose = drive.getPose();
                    ChassisSpeeds speed = new ChassisSpeeds(sample.vx + xPid.calculate(pose.getX(), sample.x),
                            sample.vy + yPid.calculate(pose.getY(), sample.y),
                            sample.omega + rPid.calculate(pose.getRotation().getRadians(), sample.heading));
                    DogLog.log("Auto/DesiredPose", sample.getPose());
                    DogLog.log("Auto/DesiredSpeed", speed);
                    drive.fieldRelativeDrive(speed);
                }, true, drive);

        chooser = new SendableChooser<>();
        chooser.setDefaultOption("NONE", Commands.none());

        this.dashboard = dashboard;

        Shuffleboard.getTab("Robot").add("Auto Selector", chooser);
    }

    public void bindAutoDefault(String name, Function<AutoFactory, AutoRoutine> autoBuilder, String description)
    {
        dashboard.putDefaultAutonomousCommand(name, description, autoBuilder.apply(factory).cmd());
        chooser.setDefaultOption(name, autoBuilder.apply(factory).cmd());
    }

    public void bindAutoDefault(String name, Command cmd, String description)
    {
        dashboard.putDefaultAutonomousCommand(name, description, cmd);
        chooser.setDefaultOption(name, cmd);
    }

    public void bindAuto(String name, Function<AutoFactory, AutoRoutine> autoBuilder, String description)
    {
        dashboard.putAutonomousCommand(name, description, autoBuilder.apply(factory).cmd());
        chooser.addOption(name, autoBuilder.apply(factory).cmd());
    }

    public void bindAuto(String name, Command cmd, String description)
    {
        dashboard.putAutonomousCommand(name, description, cmd);
        chooser.addOption(name, cmd);
    }

    public Command getSelected()
    {
        return dashboard.getSelectedAutonomousCommand();
    }

}
