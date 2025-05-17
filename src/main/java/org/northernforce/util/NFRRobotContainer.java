package org.northernforce.util;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * A robot container should contain all subsystems and states. Has various
 * utility functions.
 */
public interface NFRRobotContainer
{
    /**
     * Bind the driver OI to the commands
     */
    public void bindDriverOI();

    /**
     * Bind the programmer OI to the commands
     */
    public void bindProgrammerOI();

    /** Runs periodically (every 20 ms) regardless of mode. */
    public default void periodic()
    {
    }

    /** Runs periodically (every 20 ms) in only teleop. */
    public default void teleopPeroidic()
    {
    }

    /** Runs periodically (every 20 ms) in only autonomous. */
    public default void autonomousPeriodic()
    {
    }

    /** Runs at the start of autonomous */
    public default void autonomousInit()
    {
    }

    /** Runs at the start of disabled */
    public default void disabledInit()
    {
    }

    /** Runs at the start of teleop */
    public default void teleopInit()
    {
    }

    /** Runs at the start of test */
    public default void testInit()
    {
    }

    /**
     * Get the selected autonomous command
     * 
     * @return the selected autonomous command
     */
    public Command getAutonomousCommand();
}
