// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.zippy;

import static edu.wpi.first.units.Units.*;

import org.northernforce.util.NFRRobotContainer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.zippy.generated.ZippyTunerConstants;

/**
 * This is the main robot container for the Zippy robot. It contains the robot's
 * subsystems, commands, and OI (operator interface) bindings.
 */
public class ZippyContainer implements NFRRobotContainer
{
    private double MaxSpeed = ZippyTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /**
     * This is the swerve drivetrain subsystem for the Zippy robot. It is
     * responsible for controlling the robot's movement and handling the swerve
     * drive mechanism.
     */
    public final CommandSwerveDrivetrain drivetrain = ZippyTunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /**
     * Creates a new ZippyContainer. This is where the robot's subsystems, commands,
     * and OI (operator interface) bindings are created.
     */
    public ZippyContainer()
    {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        bindOI();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
        drivetrain.registerTelemetry(logger::telemeterize);
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
