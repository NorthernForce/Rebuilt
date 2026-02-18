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
            double xOffset = fuelDetector.getFuelBlobXOffsets().in(Radians);
            double yOffset = fuelDetector.getFuelBlobYOffsets().in(Degrees);
            double turnSpeed = config.turnSpeedMultiplier() * -MathUtil
                    .clamp(Math.pow(((config.zeroXAngle().in(Degrees) - yOffset) / config.zeroXAngle().in(Degrees)),
                            config.turnExponent()), 0, 1); // P-gain to turn toward fuel
            double forwardSpeed = config.forwardSpeedMultiplier() * -MathUtil
                    .clamp(Math.pow(((config.zeroAngle().in(Degrees) - yOffset) / config.zeroAngle().in(Degrees)),
                            config.forwardExponent()), 0, 1); // Drive
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
