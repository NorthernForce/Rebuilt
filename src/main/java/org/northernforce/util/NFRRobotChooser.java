package org.northernforce.util;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;

/**
 * A robot chooser uses the id of the roborio to choose the correct container
 * for the robot. Note: if the roborio is changed between robots, code may also
 * need to be changed.
 */
public class NFRRobotChooser
{
    protected final Supplier<NFRRobotContainer> defaultRobot;
    protected final Map<String, Supplier<NFRRobotContainer>> otherRobots;

    /**
     * Creates a new robot chooser.
     *
     * @param defaultRobot default robot container should the file be nonexistant.
     *                     Should be competition bot for purposes of fallbacks
     *                     should anything happen to the roboRio.
     * @param otherRobots  the map of roborio ids to robots (you may include the
     *                     default robot).
     */
    public NFRRobotChooser(Supplier<NFRRobotContainer> defaultRobot,
            Map<String, Supplier<NFRRobotContainer>> otherRobots)
    {
        this.defaultRobot = defaultRobot;
        this.otherRobots = otherRobots;
    }

    /**
     * Gets the Roborio ID
     * 
     * @return the id of the roborio (aka the serial number)
     */
    public static String getRoborioID()
    {
        return RobotController.getSerialNumber();
    }

    /**
     * Gets the current robot container with the matching roborio id.
     *
     * @return the robot container using one of the suppliers.
     */

    public NFRRobotContainer getNFRRobotContainer()
    {
        final var roborioID = getRoborioID();
        return otherRobots.getOrDefault(roborioID, defaultRobot).get();
    }
}
