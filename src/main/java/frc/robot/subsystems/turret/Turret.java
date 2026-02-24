package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.shooter.ShooterIO;
import frc.robot.subsystems.turret.suzie.SuzieIO;
import frc.robot.util.TargetingCalculator;

public class Turret extends SubsystemBase
{
    private final TurretConstants constants;
    private final SuzieIO suzie;
    private final HoodIO hood;
    private final ShooterIO shooter;
    private final Distance shooterOffsetDistance;
    private final TargetingCalculator hoodCalculator;
    private final TargetingCalculator shooterCalculator;

    public Turret(TurretConstants constants, SuzieIO suzie, HoodIO hood, ShooterIO shooter,
            TargetingCalculator hoodCalculator, TargetingCalculator shooterCalculator)
    {
        this.constants = constants;
        this.suzie = suzie;
        this.hood = hood;
        this.shooter = shooter;
        shooterOffsetDistance = Meters.of(constants.offset().getTranslation().getDistance(Translation2d.kZero));
        this.hoodCalculator = hoodCalculator;
        this.shooterCalculator = shooterCalculator;
    }

    public static record TurretConstants(Pose2d offset) {
    }

    public static record TurretPose(Angle suzieAngle, Angle hoodAngle, AngularVelocity shooterSpeed) {
        public static TurretPose kZero = new TurretPose(Radians.of(0), Radians.of(0), RotationsPerSecond.of(0));
    }

    /**
     * Get the current alliance
     */
    public Alliance getAlliance()
    {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /**
     * Get the hub position for the current alliance
     */
    public Translation3d getHubPosition()
    {
        return getAlliance() == Alliance.Red ? FieldConstants.kRedHubPosition : FieldConstants.kBlueHubPosition;
    }

    /**
     * Get the passing target for the current alliance
     */
    public Translation2d getPassingTarget()
    {
        return getAlliance() == Alliance.Red ? FieldConstants.kRedPassingTarget : FieldConstants.kBluePassingTarget;
    }

    /**
     * Get the center trench position for the current alliance (used for shooting
     * range calculation)
     */
    public Translation2d getCenterTrench()
    {
        return getAlliance() == Alliance.Red ? FieldConstants.kCenterRedTrench : FieldConstants.kCenterBlueTrench;
    }

    /**
     * Get the alliance station X position
     */
    public double getAllianceStationX()
    {
        return getAlliance() == Alliance.Red ? FieldConstants.kRedAllianceStationX
                : FieldConstants.kBlueAllianceStationX;
    }

    /**
     * Check if the robot is in valid shooting range for the current alliance
     * Shooting range is between the center trench and the alliance station
     */
    public boolean isInShootingRange(Pose2d robotPose)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        Translation2d hubPosition2d = new Translation2d(getHubPosition().getX(), getHubPosition().getY());
        double distanceToHub = shooterPosition.getDistance(hubPosition2d);

        // Check if within max shooting distance
        if (distanceToHub > FieldConstants.kMaxShootingDistance)
        {
            return false;
        }

        // Check if between center trench and alliance station
        Translation2d centerTrench = getCenterTrench();
        double allianceStationX = getAllianceStationX();

        if (getAlliance() == Alliance.Red)
        {
            // Red: shooter X must be between trench X and alliance station (right side of
            // field)
            return shooterPosition.getX() >= centerTrench.getX() && shooterPosition.getX() <= allianceStationX;
        } else
        {
            // Blue: shooter X must be between alliance station and trench X (left side of
            // field)
            return shooterPosition.getX() >= allianceStationX && shooterPosition.getX() <= centerTrench.getX();
        }
    }

    public void setTargetPose(TurretPose pose)
    {
        suzie.setTargetAngle(pose.suzieAngle);
        hood.setTargetAngle(pose.hoodAngle);
        shooter.setTargetSpeed(pose.shooterSpeed);
    }

    public TurretPose calculateTargetPose(Pose2d robotPose)
    {
        boolean inRange = isInShootingRange(robotPose);
        DogLog.log("Turret/InShootingRange", inRange);

        if (inRange)
        {
            // Target the hub for shooting
            return calculateShootingPose(robotPose);
        } else
        {
            // Target the passing zone
            return calculatePassingPose(robotPose);
        }
    }

    /**
     * Calculate pose for shooting at the hub
     */
    private TurretPose calculateShootingPose(Pose2d robotPose)
    {
        Distance shooterDistanceToHub = calculateShooterDistanceToHub(robotPose);
        Angle suzieAngle = calculateSuzieTargetAngle(robotPose, getHubPosition());
        Angle hoodAngle = Radians.of(hoodCalculator.getValueForDistance(shooterDistanceToHub.in(Meters)));
        AngularVelocity shooterSpeed = RotationsPerSecond
                .of(shooterCalculator.getValueForDistance(shooterDistanceToHub.in(Meters)));

        DogLog.log("Turret/TargetMode", "Shooting");
        DogLog.log("Turret/TargetPosition", new Translation2d(getHubPosition().getX(), getHubPosition().getY()));

        return new TurretPose(suzieAngle, hoodAngle, shooterSpeed);
    }

    /**
     * Calculate pose for passing to alliance corner
     */
    private TurretPose calculatePassingPose(Pose2d robotPose)
    {
        Translation2d passingTarget = getPassingTarget();
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        double distanceToTarget = shooterPosition.getDistance(passingTarget);

        // Calculate angle to passing target
        Angle suzieAngle = calculateSuzieTargetAngle(robotPose,
                new Translation3d(passingTarget.getX(), passingTarget.getY(), 0));

        // Use a lower, flatter trajectory for passing
        Angle hoodAngle = Degrees.of(5); // Low angle for passing
        AngularVelocity shooterSpeed = RotationsPerSecond.of(50); // Medium speed for passing

        DogLog.log("Turret/TargetMode", "Passing");
        DogLog.log("Turret/TargetPosition", passingTarget);
        DogLog.log("Turret/DistanceToPassingTarget", distanceToTarget);

        return new TurretPose(suzieAngle, hoodAngle, shooterSpeed);
    }

    private Angle calculateSuzieTargetAngle(Pose2d robotPose, Translation3d targetPosition)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        double targetAngle = Math.atan2(targetPosition.getY() - shooterPosition.getY(),
                targetPosition.getX() - shooterPosition.getX()) - robotPose.getRotation().getRadians()
                - constants.offset().getRotation().getRadians();
        return Radians.of(normalizeAngle(targetAngle));
    }

    /**
     * Normalize angle to be within -PI to PI
     */
    private double normalizeAngle(double angle)
    {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    public Translation2d calculateFieldRelativeShooterPosition(Pose2d robotPose)
    {
        return new Translation2d(
                robotPose.getX() + shooterOffsetDistance.in(Meters)
                        * Math.cos(robotPose.getRotation().getRadians() + Math.PI / 4),
                robotPose.getY() + shooterOffsetDistance.in(Meters)
                        * Math.sin(robotPose.getRotation().getRadians() + Math.PI / 4));
    }

    private Distance calculateShooterDistanceToHub(Pose2d robotPose)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        Translation3d hubPos = getHubPosition();
        Translation2d hubPosition2d = new Translation2d(hubPos.getX(), hubPos.getY());
        return Meters.of(shooterPosition.getDistance(hubPosition2d));
    }

    /**
     * Get the distance from the shooter to the hub for logging/debugging
     */
    public double getDistanceToHub(Pose2d robotPose)
    {
        return calculateShooterDistanceToHub(robotPose).in(Meters);
    }

    public SuzieIO getSuzie()
    {
        return suzie;
    }

    public HoodIO getHood()
    {
        return hood;
    }

    public ShooterIO getShooter()
    {
        return shooter;
    }

    public TargetingCalculator getHoodTargetingCalculator()
    {
        return hoodCalculator;
    }

    public TargetingCalculator getShooterTargetingCalculator()
    {
        return shooterCalculator;
    }

    /**
     * Check if the turret position is within danger proximity of any trench
     */
    public boolean isInDangerProximity(Translation2d turretPosition, Distance dangerZone,
            java.util.List<Translation2d> trenchPositions)
    {
        for (Translation2d trench : trenchPositions)
        {
            if (turretPosition.getDistance(trench) < dangerZone.in(Meters))
            {
                return true;
            }
        }
        return false;
    }

    public Command runBasedOnLocation(Supplier<Pose2d> robotPose, Distance dangerZone,
            java.util.List<Translation2d> trenchPositions)
    {
        return run(() ->
        {
            // Use turret/shooter position instead of robot pose
            Translation2d turretPosition = calculateFieldRelativeShooterPosition(robotPose.get());
            boolean inDanger = isInDangerProximity(turretPosition, dangerZone, trenchPositions);
            DogLog.log("Turret/DefaultCommand/TurretPosition", turretPosition);
            DogLog.log("Turret/DefaultCommand/InDanger", inDanger);
            DogLog.log("Turret/DefaultCommand/Alliance", getAlliance().toString());
            if (inDanger)
            {
                // Duck the hood when in danger zone
                hood.setTargetAngle(Degrees.of(0));
            }
            // When not in danger, keep hood at current position (don't change it)
            // Other commands like PrepTurretCommand will set the proper angle
        });
    }

    @Override
    public void periodic()
    {
        suzie.update();
        hood.update();
        shooter.update();
        DogLog.log("Turret/Suzie/Angle", suzie.getAngle().in(Radians));
        DogLog.log("Turret/Suzie/TargetAngle", suzie.getTargetAngle().in(Radians));
        DogLog.log("Turret/Suzie/IsAtTarget", suzie.isAtTargetAngle());
        DogLog.log("Turret/Hood/Angle", hood.getAngle().in(Radians));
        DogLog.log("Turret/Hood/TargetAngle", hood.getTargetAngle().in(Radians));
        DogLog.log("Turret/Hood/IsAtTarget", hood.isAtTargetAngle());
        DogLog.log("Turret/Alliance", getAlliance().toString());
    }
}