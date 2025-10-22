// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.Map;

import org.northernforce.util.NFRRobotChooser;
import org.northernforce.util.NFRRobotContainer;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.ralph.RalphContainer;

public class Robot extends TimedRobot
{
    private Command m_autonomousCommand;

    private final NFRRobotContainer m_robotContainer;

    public Robot()
    {
        DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureNt(true));
        NFRRobotChooser chooser = new NFRRobotChooser(() -> new RalphContainer(), Map.of());
        m_robotContainer = chooser.getNFRRobotContainer();
        m_robotContainer.bindOI();
        for (Field field : BuildConstants.class.getFields())
        {
            try
            {
                NetworkTableInstance.getDefault().getStringTopic("/Metadata/" + field.getName()).publish()
                        .set(field.get(null).toString());
            } catch (Exception e)
            {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        m_robotContainer.periodic();
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
