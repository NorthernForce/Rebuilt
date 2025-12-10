package frc.robot.drew;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.drew.generated.DrewTunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIO;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOLimelight;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIOPhotonVisionSim;
import frc.robot.util.AutoUtil;

public class DrewContainer implements NFRRobotContainer
{
    private final CommandSwerveDrivetrain drive;
    private final AprilTagVisionIO vision;
    private final AutoUtil autoUtil;
    private final Field2d field;

    public DrewContainer()
    {
        drive = new CommandSwerveDrivetrain(DrewTunerConstants.DrivetrainConstants,
                DrewConstants.DrivetrainConstants.kMaxSpeed, DrewConstants.DrivetrainConstants.kMaxAngularSpeed,
                DrewTunerConstants.FrontLeft, DrewTunerConstants.FrontRight, DrewTunerConstants.BackLeft,
                DrewTunerConstants.BackRight);
        if (Utils.isSimulation())
        {
            // TODO: get camera json config for sim
            vision = new AprilTagVisionIOPhotonVisionSim(
                    DrewConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                    DrewConstants.CameraConstants.kCenterCameraTransform);
        } else
        {
            vision = new AprilTagVisionIOLimelight(DrewConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                    DrewConstants.CameraConstants.kFrontRightCameraTransform,
                    DrewConstants.VisionConstants.LimeLightConstants.kValidIds);
        }
        field = new Field2d();

        autoUtil = new AutoUtil(drive, DrewConstants.AutoConstants.xPid, DrewConstants.AutoConstants.yPid,
                DrewConstants.AutoConstants.rPid);
        autoUtil.bindAutoDefault("TestAuto", this::testAuto);

        Shuffleboard.getTab("Developer").add(field);
        Shuffleboard.getTab("Developer").add("Reset Encoders", drive.resetEncoders());
        Shuffleboard.getTab("Developer").add("Reset Orientation", drive.resetOrientation());
        Shuffleboard.getTab("Developer").add("Drive to Blue Reef",
                drive.navigateToPose(new Pose2d(3, 4, new Rotation2d())));
    }

    /* gets the drive subsystem
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
        vision.getPoses().forEach(m -> drive.addVisionMeasurement(m.pose(), m.timestamp()));
        field.setRobotPose(drive.getState().Pose);
        DogLog.log("BatteryVoltage", RobotController.getBatteryVoltage());
    }

    @Override
    public void bindOI()
    {
        new DrewOI().bind(this);
    }

    @Override
    public Command getAutonomousCommand()
    {
        return autoUtil.getSelected();
    }

    public AutoRoutine testAuto(AutoFactory factory)
    {
        var routine = factory.newRoutine("TestAuto");

        var testPath = routine.trajectory("TestPath");
        var testPathReturn = routine.trajectory("TestPathReturn");

        routine.active().onTrue(Commands.sequence(testPath.resetOdometry(), testPath.cmd(),
                Commands.runOnce(() -> System.out.println("RETURNING")), testPathReturn.cmd()));

        return routine;
    }
}
