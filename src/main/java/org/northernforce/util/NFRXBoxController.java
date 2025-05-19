package org.northernforce.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is a wrapper around the CommandXboxController class that adds a
 * deadband to the joystick inputs and negates and squares the input values to
 * make them more intuitive for driving.
 */
public class NFRXBoxController extends CommandXboxController
{
    private final double deadband;

    /**
     * Creates a new NFRXBoxController with the given port and deadband.
     * 
     * @param port     USB port index for the controller (in DS)
     * @param deadband Deadband for the controller. This is the amount of stick
     *                 movement before the controller will register a value. This is
     *                 useful for eliminating noise in the controller.
     */
    public NFRXBoxController(int port, double deadband)
    {
        super(port);
        this.deadband = deadband;
    }

    /**
     * Creates a new NFRXBoxController with the given port and a default deadband of
     * 0.1.
     * 
     * @param port USB port index for the controller (in DS)
     */
    public NFRXBoxController(int port)
    {
        this(port, 0.1);
    }

    private double processJoystick(double value)
    {
        value = MathUtil.applyDeadband(value, deadband);
        return -value * Math.abs(value);
    }

    /**
     * Returns the left Y axis value of the controller with a deadband applied. This
     * is useful for eliminating noise in the controller. Also negates and squares
     * the input value to make it more intuitive for driving.
     * 
     * @return The left Y axis value of the controller with a deadband applied.
     */
    @Override
    public double getLeftY()
    {
        return processJoystick(super.getLeftX());
    }

    /**
     * Returns the left X axis value of the controller with a deadband applied. This
     * is useful for eliminating noise in the controller. Also negates and squares
     * the input value to make it more intuitive for driving.
     * 
     * @return The left X axis value of the controller with a deadband applied.
     */
    @Override
    public double getLeftX()
    {
        return processJoystick(super.getLeftY());
    }

    /**
     * Returns the right Y axis value of the controller with a deadband applied.
     * This is useful for eliminating noise in the controller. Also negates and
     * squares the input value to make it more intuitive for driving.
     * 
     * @return The right Y axis value of the controller with a deadband applied.
     */
    @Override
    public double getRightY()
    {
        return processJoystick(super.getRightX());
    }

    /**
     * Returns the right X axis value of the controller with a deadband applied.
     * This is useful for eliminating noise in the controller. Also negates and
     * squares the input value to make it more intuitive for driving.
     * 
     * @return The right X axis value of the controller with a deadband applied.
     */
    @Override
    public double getRightX()
    {
        return processJoystick(super.getRightY());
    }
}
