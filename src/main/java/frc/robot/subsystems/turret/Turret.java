package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.lobby.LobbyConstants;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.shooter.ShooterIO;
import frc.robot.subsystems.turret.suzie.SuzieIO;
import frc.robot.util.InterpolatedTargetingCalculator;
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

    public Turret(TurretConstants constants, SuzieIO suzie, HoodIO hood, ShooterIO shooter)
    {
        this.constants = constants;
        this.suzie = suzie;
        this.hood = hood;
        this.shooter = shooter;
        shooterOffsetDistance = Meters.of(constants.offset().getTranslation().getDistance(Translation2d.kZero));
        hoodCalculator = new InterpolatedTargetingCalculator(LobbyConstants.Turret.Hood.kTargetingDataFilepath);
        shooterCalculator = new InterpolatedTargetingCalculator(LobbyConstants.Turret.Shooter.kTargetingDataFilepath);
    }

    public static record TurretConstants(Pose2d offset) {
    }

    public static record TurretPose(Angle suzieAngle, Angle hoodAngle, AngularVelocity shooterSpeed) {
        public static TurretPose kZero = new TurretPose(Radians.of(0), Radians.of(0), RotationsPerSecond.of(0));
    }

    public void setTargetPose(TurretPose pose)
    {
        suzie.setTargetAngle(pose.suzieAngle);
        hood.setTargetAngle(pose.hoodAngle);
        shooter.setTargetSpeed(pose.shooterSpeed);
    }

    public TurretPose calculateTargetPose(Pose2d robotPose)
    {
        Distance shooterDistanceToHub = calculateShooterDistanceToHub(robotPose);
        Angle suzieAngle = calculateSuzieTargetAngle(robotPose);
        Angle hoodAngle = Radians.of(hoodCalculator.getValueForDistance(shooterDistanceToHub.in(Meters)));
        AngularVelocity shooterSpeed = RotationsPerSecond
                .of(shooterCalculator.getValueForDistance(shooterDistanceToHub.in(Meters)));
        return new TurretPose(suzieAngle, hoodAngle, shooterSpeed);
    }

    private Angle calculateSuzieTargetAngle(Pose2d robotPose)
    {
        Translation2d shooterPosition = calculateFieldRelativeShooterPosition(robotPose);
        double targetAngle = Math.atan2(FieldConstants.HubPosition.getY() - shooterPosition.getY(),
                FieldConstants.HubPosition.getX() - shooterPosition.getX()) - robotPose.getRotation().getRadians()
                - constants.offset().getRotation().getRadians();
        return Radians.of(targetAngle % (2 * Math.PI));
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
        return Meters.of(robotPose.getTranslation().getDistance(calculateFieldRelativeShooterPosition(robotPose)));
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
    }
}