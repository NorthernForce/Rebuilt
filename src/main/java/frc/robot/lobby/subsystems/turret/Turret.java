package frc.robot.lobby.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.lobby.subsystems.turret.hood.Hood;
import frc.robot.lobby.subsystems.turret.shooter.Shooter;
import frc.robot.lobby.subsystems.turret.suzie.Suzie;
import frc.robot.util.TargetingCalculator;

public class Turret extends SubsystemBase
{
    private Pose2d offset;
    private final Suzie suzie;
    private final Hood hood;
    private final Shooter shooter;
    private final Distance shooterOffsetDistance;
    private final TargetingCalculator hoodCalculator;
    private final TargetingCalculator shooterCalculator;
    private final Angle shooterToRobotAngle;

    public Turret(TurretConstants constants, Suzie suzie, Hood hood, Shooter shooter,
            TargetingCalculator hoodCalculator, TargetingCalculator shooterCalculator)
    {
        offset = constants.offset();
        this.suzie = suzie;
        this.hood = hood;
        this.shooter = shooter;
        shooterOffsetDistance = Meters.of(constants.offset().getTranslation().getDistance(Translation2d.kZero));
        this.hoodCalculator = hoodCalculator;
        this.shooterCalculator = shooterCalculator;
        shooterToRobotAngle = Radians
                .of(Math.atan2(constants.offset().getTranslation().getY(), constants.offset().getTranslation().getX()));

        DogLog.tunable("Turret/Offset Angle Degrees", offset.getRotation().getDegrees(), newAngle ->
        {
            setOffsetAngle(Degrees.of(newAngle));
        });
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
    public Translation2d getPassingTarget(Pose2d robotPose)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        boolean right = shooterPosition.getMeasureY().lt(Meters.of(FieldConstants.kFieldWidthMeters / 2.0));
        if (getAlliance() == Alliance.Blue)
        {
            if (right)
            {
                return FieldConstants.kBlueRightPassingTarget;
            } else
            {
                return FieldConstants.kBlueLeftPassingTarget;
            }
        } else
        {
            if (right)
            {
                return FieldConstants.kRedRightPassingTarget;
            } else
            {
                return FieldConstants.kRedLeftPassingTarget;
            }
        }
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

    public boolean isInAllianceZone(Pose2d robotPose)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);

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
        if (distanceToHub > FieldConstants.kMaxShootingDistance || distanceToHub < FieldConstants.kMinShootingDistance)
        {
            return false;
        }

        return true;
    }

    public void setTargetPose(TurretPose pose)
    {
        DogLog.log("Target Angle Being Set", pose.hoodAngle);
        suzie.setTargetAngle(pose.suzieAngle);
        hood.setTargetMechanismAngle(pose.hoodAngle);
        shooter.setTargetSpeed(pose.shooterSpeed);
    }

    public TurretPose getPose()
    {
        return new TurretPose(suzie.getAngle(), hood.getAngle(), shooter.getSpeed());
    }

    public void setTargetPoseStupid(TurretPose pose)
    {
        DogLog.log("Target Angle Being Set", pose.hoodAngle);
        hood.setTargetMechanismAngle(pose.hoodAngle);
        shooter.setTargetSpeed(pose.shooterSpeed);
    }

    public void start()
    {
        suzie.start();
        hood.start();
        shooter.start();
    }

    public void stop()
    {
        suzie.stop();
        hood.stop();
        shooter.stop();
    }

    public TurretPose calculateTargetPose(Translation2d predictedTurretPosition, Pose2d robotPose)
    {
        boolean inAllianceZone = isInAllianceZone(robotPose);
        boolean inRange = isInShootingRange(robotPose);
        DogLog.log("Turret/InAllianceZone", inAllianceZone);
        DogLog.log("Turret/InShootingRange", inRange);

        if (inAllianceZone)
        {
            if (inRange)
            {
                // Target the hub for shooting
                return calculateShootingPose(robotPose, predictedTurretPosition);
            } else
            {
                return getPose();
            }
        } else
        {
            return calculatePassingPose(robotPose, predictedTurretPosition);
        }
    }

    private TurretPose calculateShootingPose(Pose2d robotPose, Translation2d predictedTurretPosition)
    {
        Distance shooterDistanceToHub = Meters
                .of(predictedTurretPosition.getDistance(getHubPosition().toTranslation2d()));
        Angle suzieAngle = calculateSuzieTargetAngle(robotPose.getRotation().getMeasure(), predictedTurretPosition,
                getHubPosition());
        Angle hoodAngle = Degrees.of(hoodCalculator.getValueForDistance(shooterDistanceToHub.in(Meters)));
        AngularVelocity shooterSpeed = RotationsPerSecond
                .of(shooterCalculator.getValueForDistance(shooterDistanceToHub.in(Meters)));

        DogLog.log("Turret/TargetMode", "Shooting");
        DogLog.log("Turret/TargetPosition", new Translation2d(getHubPosition().getX(), getHubPosition().getY()));

        return new TurretPose(suzieAngle, hoodAngle, shooterSpeed);
    }

    /**
     * Calculate pose for passing to alliance corner
     */
    private TurretPose calculatePassingPose(Pose2d robotPose, Translation2d predictedTurretPosition)
    {
        Translation2d passingTarget = getPassingTarget(robotPose);
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        double distanceToTarget = shooterPosition.getDistance(passingTarget);

        // Calculate angle to passing target
        Angle suzieAngle = calculateSuzieTargetAngle(robotPose.getRotation().getMeasure(), predictedTurretPosition,
                new Translation3d(passingTarget.getX(), passingTarget.getY(), 0));

        Angle hoodAngle = Degrees.of(hoodCalculator.getValueForDistance(distanceToTarget));
        AngularVelocity shooterSpeed = RotationsPerSecond.of(shooterCalculator.getValueForDistance(distanceToTarget));

        DogLog.log("Turret/TargetMode", "Passing");
        DogLog.log("Turret/TargetPosition", passingTarget);
        DogLog.log("Turret/DistanceToPassingTarget", distanceToTarget);

        return new TurretPose(suzieAngle, hoodAngle, shooterSpeed);
    }

    private Angle calculateSuzieTargetAngle(Angle robotAngle, Translation2d predictedTurretPosition,
            Translation3d targetPosition)
    {
        double targetAngle = Math.atan2(targetPosition.getY() - predictedTurretPosition.getY(),
                targetPosition.getX() - predictedTurretPosition.getX()) - robotAngle.in(Radians)
                - offset.getRotation().getRadians();
        return Radians.of(normalizeAngle(targetAngle));
    }

    public void setOffsetAngle(Angle angle)
    {
        offset = new Pose2d(offset.getTranslation(), new Rotation2d(angle));
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

    public boolean isAtTargetPose()
    {
        return suzie.isAtTargetAngle() && shooter.isAtTargetSpeed();
    }

    public Translation2d calculateFieldRelativeShooterPosition(Pose2d robotPose)
    {
        return new Translation2d(
                robotPose.getX() + shooterOffsetDistance.in(Meters)
                        * Math.cos(robotPose.getRotation().getRadians() + shooterToRobotAngle.in(Radians)),
                robotPose.getY() + shooterOffsetDistance.in(Meters)
                        * Math.sin(robotPose.getRotation().getRadians() + shooterToRobotAngle.in(Radians)));
    }

    public Distance calculateShooterDistanceToHub(Pose2d robotPose)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        Translation3d hubPos = getHubPosition();
        Translation2d hubPosition2d = new Translation2d(hubPos.getX(), hubPos.getY());
        return Meters.of(shooterPosition.getDistance(hubPosition2d));
    }

    public Distance calculateShooterDistanceToHubWithTurret(Pose2d turretPose)
    {
        Translation2d shooterPosition = turretPose.getTranslation();
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

    public Suzie getSuzie()
    {
        return suzie;
    }

    public Angle getSuzieAngleRobotRelative()
    {
        return suzie.getAngle().plus(offset.getRotation().getMeasure());
    }

    public Angle getSuzieTargetAngleRobotRelative()
    {
        return suzie.getTargetAngle().plus(offset.getRotation().getMeasure());
    }

    public Hood getHood()
    {
        return hood;
    }

    public Shooter getShooter()
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
        DogLog.log("Turret/Suzie/Angle", suzie.getAngle().in(Degrees));
        DogLog.log("Turret/Suzie/TargetAngle", suzie.getTargetAngle().in(Degrees));
        DogLog.log("Turret/Suzie/IsAtTarget", suzie.isAtTargetAngle());
        DogLog.log("Turret/Hood/Angle", hood.getAngle().in(Radians));
        DogLog.log("Turret/Hood/TargetAngle", hood.getTargetAngle().in(Radians));
        DogLog.log("Turret/Hood/IsAtTarget", hood.isAtTargetAngle());
    }
}