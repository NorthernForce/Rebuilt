package frc.robot.ralph;

import java.io.File;
import java.util.stream.Stream;

import org.northernforce.util.NFRRobotContainer;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AutoUtil;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Field2d field;

    public RalphContainer()
    {
        drive = new CommandSwerveDrivetrain(RalphTunerConstants.DrivetrainConstants,
                RalphConstants.DrivetrainConstants.kMaxSpeed, RalphConstants.DrivetrainConstants.kMaxAngularSpeed,
                RalphTunerConstants.FrontLeft, RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft,
                RalphTunerConstants.BackRight);
        field = new Field2d();
        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());
        AutoUtil.buildAutos();
    }

    /**
     * gets the drive subsystem
     *
     * @return the drive subsystem
     */
    public CommandSwerveDrivetrain getDrive()
    {
        return drive;
    }

    @Override
    public void periodic()
    {
        field.setRobotPose(getDrive().getState().Pose);
    }

    @Override
    public void bindOI()
    {
        new RalphOI().bind(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return AutoUtil.getSelected();
    }

}
