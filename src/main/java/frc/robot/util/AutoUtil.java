package frc.robot.util;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoUtil
{
    private final static SendableChooser<Command> chooser = new SendableChooser<>();

    public static void buildAutos(CommandSwerveDrivetrain drive, PIDController xPid, PIDController yPid,
            PIDController rPid)
    {
        var factory = new AutoFactory(() -> drive.getState().Pose, drive::resetPose, (SwerveSample sample) ->
        {
            var pose = drive.getState().Pose;
            ChassisSpeeds speed = new ChassisSpeeds(sample.vx + xPid.calculate(pose.getX(), sample.x),
                    sample.vy + yPid.calculate(pose.getY(), sample.y),
                    sample.omega + rPid.calculate(pose.getRotation().getRadians(), sample.heading));
            drive.relativeDrive(speed);
        }, true, drive);

        chooser.setDefaultOption("Nothing", Commands.none());
        chooser.addOption("TestPath",
                Commands.sequence(factory.resetOdometry("TestPath"), factory.trajectoryCmd("TestPath"),
                        Commands.runOnce(() -> System.out.println("RETURNING")),
                        factory.trajectoryCmd("TestPathReturn")));

        Shuffleboard.getTab("Robot").add("Auto Selector", chooser);
    }

    public static Command getSelected()
    {
        return chooser.getSelected();
    }
}
