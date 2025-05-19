// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import org.northernforce.util.NFRRobotChooser;
import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.zippy.ZippyContainer;

/**
 * The main robot class. This is where the robot code starts and ends. This
 * class is responsible for creating the robot-specific container and running
 * the robot code.
 */
public class Robot extends TimedRobot
{
    private Command m_autonomousCommand;

    private final NFRRobotContainer m_robotContainer;

    /**
     * The robot container is the main class for the robot. It is responsible for
     * creating the robot-specific container and running the robot code.
     */
    public Robot()
    {
        NFRRobotChooser chooser = new NFRRobotChooser(() -> new ZippyContainer(), Map.of());
        m_robotContainer = chooser.getNFRRobotContainer();
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit()
    {
    }

    @Override
    public void disabledPeriodic()
    {
    }

    @Override
    public void disabledExit()
    {
    }

    @Override
    public void autonomousInit()
    {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void autonomousExit()
    {
    }

    @Override
    public void teleopInit()
    {
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void teleopExit()
    {
    }

    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void testExit()
    {
    }

    @Override
    public void simulationPeriodic()
    {
    }
}
