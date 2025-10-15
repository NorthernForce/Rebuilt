package frc.robot.ralph;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.ralph.generated.RalphTunerConstants;
import frc.robot.ralph.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RalphContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final Field2d field;
    private final ShooterIOTalonFX shooter;

    public RalphContainer()
    {
        shooter = new ShooterIOTalonFX(RalphConstants.ShooterConstants.kMotorId,
        RalphConstants.ShooterConstants.kMotorSpeed);
        drive = new CommandSwerveDrivetrain(RalphTunerConstants.DrivetrainConstants,
                RalphConstants.DrivetrainConstants.kMaxSpeed, RalphConstants.DrivetrainConstants.kMaxAngularSpeed,
                RalphTunerConstants.FrontLeft, RalphTunerConstants.FrontRight, RalphTunerConstants.BackLeft,
                RalphTunerConstants.BackRight);
        field = new Field2d();
        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());
        bindOI();
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

    public ShooterIOTalonFX getShooter()
    {
        return shooter;
    }

    @Override
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }

}
