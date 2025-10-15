package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ralph.RalphConstants.InnerElevatorConstants;
import frc.robot.ralph.RalphConstants.OuterElevatorConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.elevator.SimElevator;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Superstructure superstructure;
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
        if (RobotBase.isReal())
        {
            superstructure = new Superstructure();
        } else
        {
            superstructure = new Superstructure(
                    new SimElevator(InnerElevatorConstants.kCanID, InnerElevatorConstants.kSensorID,
                            InnerElevatorConstants.kConfig),
                    new SimElevator(OuterElevatorConstants.kCanID, OuterElevatorConstants.kSensorID,
                            OuterElevatorConstants.kConfig));
        }
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
        return Commands.none();
    }

    public Superstructure getSuperstructure()
    {
        return superstructure;
    }
}
