package frc.robot.util;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoUtil
{
    private final static SendableChooser<Command> chooser = new SendableChooser<>();

    public static void buildAutos(Class<?> autoClass, CommandSwerveDrivetrain drive, PIDController xPid,
            PIDController yPid, PIDController rPid)
    {
        var factory = new AutoFactory(() -> drive.getState().Pose, drive::resetPose, (SwerveSample sample) ->
        {
            var pose = drive.getState().Pose;
            ChassisSpeeds speed = new ChassisSpeeds(sample.vx + xPid.calculate(pose.getX(), sample.x),
                    sample.vy + yPid.calculate(pose.getY(), sample.y),
                    sample.omega + rPid.calculate(pose.getRotation().getRadians(), sample.heading));
            drive.fieldRelativeDrive(speed);
        }, true, drive);

        for (Method method : autoClass.getDeclaredMethods())
        {
            // ignore lambda expressions
            if (method.isSynthetic())
                continue;

            if (Arrays.equals(method.getParameterTypes(), new Class<?>[]
            { AutoFactory.class }) && method.getReturnType() == AutoRoutine.class
                    && Modifier.isStatic(method.getModifiers()) && Modifier.isPublic(method.getModifiers()))
            {
                try
                {
                    chooser.addOption(method.getName(), ((AutoRoutine) method.invoke(null, factory)).cmd());
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
                continue;
            }
            throw new IllegalArgumentException(String.format("Invalid auto method detected. (%s)\n".concat(
                    "(All methods must roughly follow the format `public static AutoRoutine autoName(AutoFactory factory)`)."),
                    method.toString(), method.getParameterTypes()[0].toString(), Stream.of(method.getParameterTypes())
                            .map(m -> m.toString()).reduce((a, b) -> a.concat(b)).orElseThrow()));

        }

        chooser.setDefaultOption("Nothing", Commands.none());
        Shuffleboard.getTab("Robot").add("Auto Selector", chooser);
    }

    public static Command getSelected()
    {
        return chooser.getSelected();
    }
}
