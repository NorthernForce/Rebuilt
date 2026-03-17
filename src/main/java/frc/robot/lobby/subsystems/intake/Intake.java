package frc.robot.lobby.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Intake extends SubsystemBase
{
    public IntakeIO io;

    private final SysIdRoutine m_armSysIdRoutine;

    private final Angle downAngle;
    private final Angle midAngle;
    private final Angle stowAngle;
    private final double intakeSpeed;
    private final double purgeSpeed;
    private final Angle angleTolerance;

    public Intake(IntakeIO io, IntakeParameters params)
    {
        this.io = io;

        m_armSysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.5).per(Second), Volts.of(6), Seconds.of(3),
                        state -> DogLog.log("IntakeArm_SysId_State", state.toString())),
                new SysIdRoutine.Mechanism(voltage -> io.setArmVoltage(voltage),
                        log -> log.motor("IntakeArm").voltage(Volts.of(io.getArmVoltage()))
                                .angularPosition(Rotations.of(io.getArmPosition()))
                                .angularVelocity(RotationsPerSecond.of(io.getArmVelocity())),
                        this));
        downAngle = params.downAngle();
        midAngle = params.midAngle();
        stowAngle = params.stowAngle();
        intakeSpeed = params.intakeSpeed();
        purgeSpeed = params.purgeSpeed();
        this.angleTolerance = params.angleTolerance();
    }

    public class AgitateCommand extends ParallelCommandGroup
    {
        public AgitateCommand()
        {
            addCommands(Commands.run(() -> io.intake(intakeSpeed)),
                    (getRunToMidAngleCommand().andThen(getRunToStowAngleCommand()).repeatedly()));
        }
    }

    public Command purgeIntake()
    {
        return run(() -> io.purgeIntake(purgeSpeed));
    }

    public Command intake()
    {
        return run(() -> io.intake(intakeSpeed));
    }

    public class RunToAngleCommand extends Command
    {
        private final Angle angle;

        public RunToAngleCommand(Angle angle)
        {
            this.angle = angle;
            addRequirements(Intake.this);

        }

        @Override
        public void initialize()
        {
            io.setAngle(angle);
        }

        @Override
        public boolean isFinished()
        {
            return Rotations.of(io.getArmPosition()).isNear(angle, angleTolerance);
        }
    }

    public class MoveAndIntake extends Command
    {
        public MoveAndIntake()
        {
            addRequirements(Intake.this);
        }

        @Override
        public void initialize()
        {
            io.setAngle(downAngle);
            io.intake(intakeSpeed);
        }
    }

    public Command getRunToIntakeAngleCommand()
    {
        return new RunToAngleCommand(downAngle);
    }

    public Command intakeMoving()
    {
        return new MoveAndIntake();
    }

    public Command getRunToStowAngleCommand()
    {

        return new RunToAngleCommand(stowAngle);
    }

    public Command getRunToMidAngleCommand()
    {

        return new RunToAngleCommand(midAngle);
    }

    public Command stopIntake()
    {
        return runOnce(() -> io.stopIntake());
    }

    public Command driveByJoystick(DoubleSupplier positionSupplier)
    {
        return run(() -> io.setPower(positionSupplier.getAsDouble()));
    }

    public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction)
    {
        return Commands.sequence(Commands.runOnce(() -> io.disableSoftLimits()),
                m_armSysIdRoutine.quasistatic(direction).finallyDo(() -> io.enableSoftLimits()));
    }

    public Command sysIdArmDynamic(SysIdRoutine.Direction direction)
    {
        return Commands.sequence(Commands.runOnce(() -> io.disableSoftLimits()),
                m_armSysIdRoutine.dynamic(direction).finallyDo(() -> io.enableSoftLimits()));
    }

    @Override
    public void periodic()
    {
        io.logArmSignals();
    }

    public double getRollerCurrent()
    {
        return io.getRollerCurrent();
    }

    public double getAnglingCurrent()
    {
        return io.getAnglingCurrent();
    }

    public record IntakeParameters(Angle downAngle, Angle midAngle, Angle stowAngle, double intakeSpeed,
            double purgeSpeed, Angle angleTolerance) {
    }
}
