package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ralph.generated.RalphTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.CTREUtil;
import frc.robot.util.Status;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem
{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final LinearVelocity maxSpeed;
    private final AngularVelocity maxAngularSpeed;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default
                                                                                                          // ramp rate
                                                                                                          // (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp
                                                                                                    // rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /*
     * SysId routine for characterizing rotation. This is used to find PID gains for
     * the FieldCentricFacingAngle HeadingController. See the documentation of
     * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI), null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(output ->
            {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            }, null, this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * just so the generated tuner code doesn't error (DONT USE NORMALLY UNLESS YOU
     * DONT WANT THE ROBOT TO MOVE)
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants,
                modules[0].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetFrontLeft", modules[0].EncoderOffset))),
                modules[1].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetFrontRight", modules[1].EncoderOffset))),
                modules[2].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetBackLeft", modules[2].EncoderOffset))),
                modules[3].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetBackRight", modules[3].EncoderOffset))));
        this.maxSpeed = LinearVelocity.ofBaseUnits(0, MetersPerSecond);
        this.maxAngularSpeed = AngularVelocity.ofBaseUnits(0, DegreesPerSecond);
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants,
                modules[0].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetFrontLeft", modules[0].EncoderOffset))),
                modules[1].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetFrontRight", modules[1].EncoderOffset))),
                modules[2].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetBackLeft", modules[2].EncoderOffset))),
                modules[3].withEncoderOffset(
                        Rotations.of(Preferences.getDouble("kSwerveOffsetBackRight", modules[3].EncoderOffset))));
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            LinearVelocity maxSpeed, AngularVelocity maxAngularSpeed, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can access
     * them through getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation in the form [x, y, theta]ᵀ, with
     *                                  units in meters and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation in the form [x, y, theta]ᵀ, with
     *                                  units in meters and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, LinearVelocity maxSpeed,
            AngularVelocity maxAngularSpeed, SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        this.maxSpeed = maxSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        if (Utils.isSimulation())
        {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder()
    {
        try
        {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(() -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this // Subsystem for
                                                                                                  // requirements
            );
        } catch (Exception ex)
        {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified
     * by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction)
    {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic()
    {
        /*
         * Periodically try to apply the operator perspective. If we haven't applied the
         * operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match. Otherwise, only check and apply the operator perspective if the DS
         * is disabled. This ensures driving behavior doesn't change until an explicit
         * disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled())
        {
            DriverStation.getAlliance().ifPresent(allianceColor ->
            {
                setOperatorPerspectiveForward(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Get a command that drives the robot by joystick input
     * 
     * @param xSupplier     x input (relative to the field)
     * @param ySupplier     y input (relative to the field)
     * @param omegaSupplier omega input (rotational rate)
     * @return a command that drives the robot by joystick input
     */
    public Command driveByJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier)
    {
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);
        return applyRequest(() ->
        {
            return request.withVelocityX(maxSpeed.times(xSupplier.getAsDouble()))
                    .withVelocityY(maxSpeed.times(ySupplier.getAsDouble()))
                    .withRotationalRate(maxAngularSpeed.times(omegaSupplier.getAsDouble()));
        });
    }

    public Command navigateToPose(Pose2d pose)
    {
        // yes infinity is okay for the acceleration values
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(maxSpeed.in(MetersPerSecond),
                Double.POSITIVE_INFINITY, maxAngularSpeed.in(RadiansPerSecond), Double.POSITIVE_INFINITY));
    }

    public Command navigateToPose(Pose2d pose, LinearVelocity limitedSpeed)
    {
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(limitedSpeed.in(MetersPerSecond),
                Double.POSITIVE_INFINITY, maxAngularSpeed.in(RadiansPerSecond), Double.POSITIVE_INFINITY));
    }

    public Command navigateToPose(Pose2d pose, LinearVelocity limitedSpeed, AngularVelocity limitedAngularSpeed)
    {
        return AutoBuilder.pathfindToPose(pose, new PathConstraints(limitedSpeed.in(MetersPerSecond),
                Double.POSITIVE_INFINITY, limitedAngularSpeed.in(RadiansPerSecond), Double.POSITIVE_INFINITY));
    }

    public Command navigateToPose(Pose2d pose, LinearVelocity limitedSpeed, LinearAcceleration limitedAcceleration)
    {
        return AutoBuilder.pathfindToPose(pose,
                new PathConstraints(limitedSpeed.in(MetersPerSecond), limitedAcceleration.in(MetersPerSecondPerSecond),
                        maxAngularSpeed.in(RadiansPerSecond), Double.POSITIVE_INFINITY));
    }

    private void startSimThread()
    {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() ->
        {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds)
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement in the form [x, y, theta]ᵀ, with
     *                                 units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs)
    {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /**
     * Resets the angle of the specified module's encoder to the specified angle.
     * This is useful for calibrating the module's encoder to a known angle.
     * 
     * @param moduleIdx   the index of the module to reset
     * @param targetAngle the angle to set the encoder to
     * @return the new offset of the encoder
     */
    private Angle resetEncoderAngle(int moduleIdx, Angle targetAngle)
    {
        final var module = getModule(moduleIdx);
        final var currentAngle = Rotations.of(module.getCurrentState().angle.getRotations());
        final var delta = targetAngle.minus(currentAngle);
        final var cancoder = module.getEncoder();
        final var config = new CANcoderConfiguration();
        cancoder.getConfigurator().refresh(config);
        final var currentOffset = Rotations.of(config.MagnetSensor.MagnetOffset);
        var newOffset = currentOffset.plus(delta);
        newOffset = Radians.of(MathUtil.angleModulus(newOffset.in(Radians)));
        config.MagnetSensor.MagnetOffset = newOffset.in(Rotations);
        cancoder.getConfigurator().apply(config);
        return newOffset;
    }

    /**
     * Resets the angle of all modules' encoders to the specified angles. This is
     * useful for calibrating the module's encoders to known angles.
     * 
     * @param targetAngles the angles to set the encoders to
     * @return the new offsets of the encoders
     */
    public Angle[] resetEncoderAngles(Angle[] targetAngles)
    {
        final var newOffsets = new Angle[targetAngles.length];
        for (int i = 0; i < targetAngles.length; i++)
        {
            newOffsets[i] = resetEncoderAngle(i, targetAngles[i]);
        }
        return newOffsets;
    }

    /**
     * Resets the angle of all modules' encoders to zero. This is useful for
     * calibrating the module's encoders to known angles. This saves the offsets to
     * the preferences so they can be retrieved later.
     */
    public void resetDriveEncoders()
    {
        final var offsets = resetEncoderAngles(new Angle[]
        { Degrees.of(0), Degrees.of(0), Degrees.of(0), Degrees.of(0) });
        Preferences.setDouble("kSwerveOffsetFrontLeft", offsets[0].in(Rotations));
        Preferences.setDouble("kSwerveOffsetFrontRight", offsets[1].in(Rotations));
        Preferences.setDouble("kSwerveOffsetBackLeft", offsets[2].in(Rotations));
        Preferences.setDouble("kSwerveOffsetBackRight", offsets[3].in(Rotations));
    }

    /**
     * Resets the angle of all modules' encoders to zero. This is useful for
     * calibrating the module's encoders to known angles.
     * 
     * @return a command that resets the encoders
     */
    public Command resetEncoders()
    {
        return Commands.runOnce(this::resetDriveEncoders, this);
    }

    /**
     * Resets orientation based on operator forward direction.
     *
     * @return a command that resets orientation
     */
    public Command resetOrientation()
    {
        return Commands.runOnce(() -> resetRotation(getOperatorForwardDirection()), this);
    }

    /**
     * Get status of a specific swerve module
     * 
     * @param idx    index of module
     * @param module the module
     * @return status of the specific swerve module
     */

    public static Status getModuleStatus(int idx, SwerveModule<TalonFX, TalonFX, CANcoder> module)
    {
        Status driveMotorStatus = CTREUtil.getTalonFXStatus(module.getDriveMotor());
        Status steerMotorStatus = CTREUtil.getTalonFXStatus(module.getSteerMotor());
        Status encoderStatus = CTREUtil.getCANcoderStatus(module.getEncoder());
        Status moduleStatus = new Status("Swerve Module " + idx + " Status", new Status[]
        { driveMotorStatus, steerMotorStatus, encoderStatus });
        return moduleStatus;
    }

    /**
     * Get status of current drive subsystem
     * 
     * @return status of current drive subsystem
     */

    public Status getStatus()
    {
        Status[] motorsStatus = new Status[getModules().length];
        for (int i = 0; i < getModules().length; i++)
        {
            motorsStatus[i] = getModuleStatus(i, getModule(i));
        }
        Status overallStatus = new Status("Command Swerve Drivetrain Status", motorsStatus);
        return overallStatus;
    }

}
