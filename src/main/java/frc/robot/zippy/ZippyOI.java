package frc.robot.zippy;

import org.northernforce.util.NFRXBoxController;

public class ZippyOI
{
    private final NFRXBoxController driverController = new NFRXBoxController(0, 0.1);

    public void bind(ZippyContainer container)
    {
        container.drivetrain.setDefaultCommand(container.drivetrain.driveByJoystick(driverController::getLeftY,
                driverController::getLeftX, driverController::getRightX));
    }
}
