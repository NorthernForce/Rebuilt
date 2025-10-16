package frc.robot.util;

import java.io.File;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoUtil
{
    private final static SendableChooser<Command> chooser = new SendableChooser<>();

    public static void buildAutos()
    {
        chooser.setDefaultOption("Nothing", Commands.none());

        NamedCommands.registerCommand("test", Commands.runOnce(() -> System.out.println("test")));

        var autoFiles = new File(Filesystem.getDeployDirectory(), "choreo").listFiles();
        (autoFiles == null ? Stream.<File>empty() : Stream.of(autoFiles)).filter(file -> file.isFile()).map(File::getName).filter(name -> name.endsWith(".traj"))
                .map(name -> name.replace(".traj", "")).forEach(name ->
                {
                    try
                    {
                        chooser.addOption(name, AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(name)));
                    } catch (Exception e)
                    {
                        e.printStackTrace();
                    }
                });
        Shuffleboard.getTab("Robot").add("Auto Selector", chooser);
    }

    public static Command getSelected()
    {
        return chooser.getSelected();
    }
}
