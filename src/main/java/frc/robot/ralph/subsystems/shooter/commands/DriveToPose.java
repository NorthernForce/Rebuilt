package frc.robot.ralph.subsystems.shooter.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command
{

    private final CommandSwerveDrivetrain swerve;
    private final Pose2d targetPose;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    private final SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds();

    @SuppressWarnings(
    { "rawtypes", "unchecked" })
    public DriveToPose(CommandSwerveDrivetrain swerve, Pose2d targetPose, Measure driveMaxSpeed, Measure driveMaxAccel,
            Measure rotationMaxSpeed, Measure rotationMaxAccel, double translationKp, double translationKi,
            double translationKd, double rotationKp, double rotationKi, double rotationKd)
    {

        this.swerve = swerve;
        this.targetPose = targetPose;

        addRequirements(swerve);

        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(
                driveMaxSpeed.in(MetersPerSecond), driveMaxAccel.in(MetersPerSecondPerSecond));
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                rotationMaxSpeed.in(RadiansPerSecond), rotationMaxAccel.in(RadiansPerSecond.per(Second)));

        xController = new ProfiledPIDController(translationKp, translationKi, translationKd, translationConstraints);
        yController = new ProfiledPIDController(translationKp, translationKi, translationKd, translationConstraints);
        thetaController = new ProfiledPIDController(rotationKp, rotationKi, rotationKd, rotationConstraints);

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

    }

    @Override
    public boolean isFinished()
    {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
}