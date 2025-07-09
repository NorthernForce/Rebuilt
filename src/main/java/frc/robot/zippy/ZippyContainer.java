// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.zippy;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import org.northernforce.util.NFRRobotContainer;
import org.photonvision.simulation.SimCameraProperties;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.apriltagvision.*;
import frc.robot.zippy.generated.ZippyTunerConstants;

public class ZippyContainer implements NFRRobotContainer
{
    private double MaxSpeed = ZippyTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandSwerveDrivetrain drivetrain = ZippyTunerConstants.createDrivetrain();

    private final AprilTagVisionIO aprilTagVisionIO;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public ZippyContainer()
    {
        if (RobotBase.isSimulation())
        {
            aprilTagVisionIO = new AprilTagVisionIOPhotonVisionSim(
                    ZippyConstants.VisionConstants.LimeLightConstants.kLimeLightName, new SimCameraProperties(),
                    ZippyConstants.VisionConstants.PhotonVisionConstants.kRobotToCamera);
        } else
        {
            aprilTagVisionIO = new AprilTagVisionIOLimelight(
                    ZippyConstants.VisionConstants.LimeLightConstants.kLimeLightName);
            LimelightHelpers.SetFiducialIDFiltersOverride(
                    ZippyConstants.VisionConstants.LimeLightConstants.kLimeLightName,
                    ZippyConstants.VisionConstants.LimeLightConstants.kValidIds);
        }
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        bindOI();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    public void periodic()
    {
        if (RobotBase.isSimulation())
        {
            AprilTagVisionIOPhotonVisionSim.getVisionSystemSim().update(drivetrain.getState().Pose);
        }
        List<Pose2dWithTimestamp> poses = aprilTagVisionIO.getPoses();
        Rotation2d robotYaw = drivetrain.getState().Pose.getRotation();
        aprilTagVisionIO.setHeading(robotYaw, Rotation2d.fromDegrees(0));
        for (Pose2dWithTimestamp pose : poses)
        {
            drivetrain.addVisionMeasurement(pose.pose(), pose.timestamp(), ZippyConstants.VisionConstants.kStdDevs);
        }

    }

    public Command getAutonomousCommand()
    {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    @Override
    public void bindOI()
    {
        new ZippyOI().bind(this);
    }
}
