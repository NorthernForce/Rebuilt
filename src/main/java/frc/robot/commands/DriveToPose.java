package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DriveToPose extends Command
{

    private final CommandSwerveDrivetrain swerve;
    private final Pose2d targetPose;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private static final double kRotationMaxVelocity = Math.PI; // Radians per second (half spin/sec)
    private static final double kRotationMaxAccel = Math.PI; // Radians per second squared

    private final SwerveRequest.ApplyChassisSpeeds driveRequest = new SwerveRequest.ApplyChassisSpeeds();

    public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d targetPose, double driveMaxSpeed, double driveMaxAccel)
    {

        this.swerve = swerve;
        this.targetPose = targetPose;

        addRequirements(swerve);

        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(driveMaxSpeed,
                driveMaxAccel);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(kRotationMaxVelocity,
                kRotationMaxAccel);

        xController = new ProfiledPIDController(1.0, 0, 0, translationConstraints);
        yController = new ProfiledPIDController(1.0, 0, 0, translationConstraints);
        thetaController = new ProfiledPIDController(1.0, 0, 0, rotationConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(0.05);

        driveRequest.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    }

    @Override
    public void initialize()
    {
        Pose2d currentPose = swerve.getState().Pose;

        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());
    }

    @Override
    public void execute()
    {
        Pose2d currentPose = swerve.getState().Pose;

        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, thetaSpeed,
                currentPose.getRotation());

        driveRequest.withSpeeds(speeds);
        swerve.setControl(driveRequest);
    }

    @Override
    public void end(boolean interrupted)
    {
        driveRequest.withSpeeds(new ChassisSpeeds(0, 0, 0));
        swerve.setControl(driveRequest);
    }

    @Override
    public boolean isFinished()
    {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}