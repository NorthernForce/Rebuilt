package frc.robot.lobby.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lobby.LobbyConstants.ClimberConstants.ClimbLevels;

public class ClimberIOTalonFX implements ClimberIO
{
    private final int motorID;
    private final TalonFX motor;
    private ClimbLevels currentLevel;
    private final Angle tolerance;

    private double setRotations = 0;
    private final Pose2d upperRedClimbPosition;
    private final Pose2d lowerRedClimbPosition;
    private final Pose2d upperBlueClimbPosition;
    private final Pose2d lowerBlueClimbPosition;
    private final double power;
    private final Angle topRotations;
    private final Angle bottomRotations;

    public ClimberIOTalonFX(ClimberParameters params)
    {
        motorID = params.motorID();
        motor = new TalonFX(motorID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = params.topSoftRotations().in(Rotations);
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = params.topSoftRotations().in(Rotations);
        config.MotorOutput.Inverted = params.inverted() ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);
        upperRedClimbPosition = params.upperRedPrepPose();
        lowerRedClimbPosition = params.lowerRedPrepPose();
        upperBlueClimbPosition = params.upperBluePrepPose();
        lowerBlueClimbPosition = params.lowerBluePrepPose();
        power = params.dutyCyclePower();
        tolerance = params.tolerance();
        topRotations = params.topSoftRotations();
        bottomRotations = params.bottomSoftRotations();
    }

    @Override
    public void runUp()
    {
        motor.set(power);
    }

    @Override
    public void homeDown()
    {
        motor.set(-power);
    }

    @Override
    public boolean atTop()
    {
        return Rotations.of(getRotations()).isNear(topRotations, tolerance);
    }

    @Override
    public boolean atBottom()
    {
        return Rotations.of(getRotations()).isNear(bottomRotations, tolerance);
    }

    @Override
    public void stopMotor()
    {
        motor.stopMotor();
    }

    public Pose2d getNearestPreclimbPosition(Pose2d robotPose)
    {
        DogLog.log("Climber/RobotPose", robotPose);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
        {
            double distanceToUpperBlue = robotPose.getTranslation()
                    .getDistance(upperBlueClimbPosition.getTranslation());
            double distanceToLowerBlue = robotPose.getTranslation()
                    .getDistance(lowerBlueClimbPosition.getTranslation());
            return distanceToUpperBlue < distanceToLowerBlue ? upperBlueClimbPosition : lowerBlueClimbPosition;
        } else if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            double distanceToUpperRed = robotPose.getTranslation().getDistance(upperRedClimbPosition.getTranslation());
            double distanceToLowerRed = robotPose.getTranslation().getDistance(lowerRedClimbPosition.getTranslation());
            return distanceToUpperRed < distanceToLowerRed ? upperRedClimbPosition : lowerRedClimbPosition;
        } else
        {
            return robotPose;
        }
    }

    @Override
    public double getRotations()
    {
        return motor.getPosition().getValueAsDouble();
    }
}
