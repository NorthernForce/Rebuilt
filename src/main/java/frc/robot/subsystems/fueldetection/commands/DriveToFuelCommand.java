package frc.robot.subsystems.fueldetection.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.fueldetection.DriveToFuelCommandConstants;
import frc.robot.subsystems.fueldetection.FuelDetector;

public class DriveToFuelCommand extends Command
{
    private final CommandSwerveDrivetrain drive;
    private final FuelDetector fuelDetector;
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final DriveToFuelCommandConstants config;

    public DriveToFuelCommand(CommandSwerveDrivetrain drive, FuelDetector fuelDetector,
            DriveToFuelCommandConstants config)
    {
        addRequirements(drive);
        this.drive = drive;
        this.fuelDetector = fuelDetector;
        this.config = config;
    }

    @Override
    public void execute()
    {
        if (fuelDetector.isFuelPresent())
        {
            double xOffset = fuelDetector.getFuelBlobXOffsets().in(Degrees);
            double yOffset = fuelDetector.getFuelBlobYOffsets().in(Degrees);
            double turnSpeed = config.turnSpeedMultiplier() * Math.pow(MathUtil
                    .clamp(Math.abs((config.zeroXAngle().in(Degrees) - xOffset) / config.turnDampening()), 0, 1),
                    config.turnExponent()) * (Math.abs(xOffset) / xOffset);
            double forwardSpeed = config.forwardSpeedMultiplier() * -Math.pow(
                    MathUtil.clamp((config.zeroAngle().in(Degrees) - yOffset) / config.forwardDampening(), 0, 1),
                    config.forwardExponent()); // Drive
            // forward
            // toward
            // fuel
            DogLog.log("FuelDetection/TurnSpeed", turnSpeed);
            DogLog.log("FuelDetection/YOffset", yOffset);
            DogLog.log("FuelDetection/XOffset", xOffset);
            DogLog.log("ForwardSpeed", forwardSpeed);
            drive.setControl(robotCentric.withVelocityX(forwardSpeed).withVelocityY(0).withRotationalRate(turnSpeed));
        } else
        {
            // No fuel detected, stop and wait
            drive.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
