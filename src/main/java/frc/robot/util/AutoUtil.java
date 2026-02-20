package frc.robot.util;

import java.util.function.Function;

import com.pathplanner.lib.commands.PathPlannerAuto;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoUtil
{
    private final AutoFactory factory;
    private final SendableChooser<Command> chooser;

    public AutoUtil(CommandSwerveDrivetrain drive, PIDController xPid, PIDController yPid, PIDController rPid)
    {
        factory = new AutoFactory(() -> drive.getState().Pose, pose -> drive.resetTranslation(pose.getTranslation()),
                (SwerveSample sample) ->
                {
                    var pose = drive.getState().Pose;
                    ChassisSpeeds speed = new ChassisSpeeds(sample.vx + xPid.calculate(pose.getX(), sample.x),
                            sample.vy + yPid.calculate(pose.getY(), sample.y),
                            sample.omega + rPid.calculate(pose.getRotation().getRadians(), sample.heading));
                    DogLog.log("Auto/DesiredPose", sample.getPose());
                    DogLog.log("Auto/DesiredSpeed", speed);
                    drive.fieldRelativeDrive(speed);
                }, true, drive);

        chooser = new SendableChooser<>();
        chooser.setDefaultOption("Nothing", Commands.none());

        Shuffleboard.getTab("Robot").add("Auto Selector", chooser);
    }

    public void bindAutoDefault(String name, Function<AutoFactory, AutoRoutine> autoBuilder)
    {
        chooser.setDefaultOption(name, autoBuilder.apply(factory).cmd());
    }

    public void bindAuto(String name, Function<AutoFactory, AutoRoutine> autoBuilder)
    {
        chooser.addOption(name, autoBuilder.apply(factory).cmd());
    }

    public void bindAuto(String name, PathPlannerAuto auto)
    {
        chooser.addOption(name, auto);
    }

    public Command getSelected()
    {
        return chooser.getSelected();
    }
}
