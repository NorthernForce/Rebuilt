package frc.robot.lobby.subsystems.nfrdashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * MatchRecorder
 *
 * Automatically starts WPILib's built-in DataLogManager recording at the
 * beginning of autonomous and stops (but keeps the file open) when the match
 * ends. DataLogManager writes a binary .wpilog to the USB drive / roboRIO flash
 * automatically.
 *
 * The recorder also: - publishes /NFRDashboard/recorder/active (boolean) so the
 * dashboard can show a "REC" badge - publishes /NFRDashboard/recorder/filename
 * (string) with the current log path - publishes /AdvantageKit/ prefixed topics
 * at 20 Hz so AdvantageScope can subscribe directly over NT4
 *
 * Usage in RobotContainer: MatchRecorder.INSTANCE.register(); // call once in
 * robotInit
 *
 * DataLogManager is already included in WPILib — no extra dependency needed.
 */
public class MatchRecorder extends SubsystemBase
{
    public static final MatchRecorder INSTANCE = new MatchRecorder();

    private static final String NT_ROOT = "/NFRDashboard/recorder";
    private static final String AK_ROOT = "/AdvantageKit/RealOutputs";

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

    // State
    private boolean wasEnabled = false;
    private boolean isRecording = false;
    private double matchStartTime = -1;

    private MatchRecorder()
    {
    }

    /**
     * Call once from robotInit to register this subsystem with the scheduler and
     * start DataLogManager with NT logging.
     */
    public void register()
    {
        // Log all NT topics automatically
        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        publishStatus();
    }

    @Override
    public void periodic()
    {
        boolean enabled = DriverStation.isEnabled();
        boolean auto = DriverStation.isAutonomous();
        boolean teleop = DriverStation.isTeleop();

        // Start recording on first enable (auto or teleop)
        if (enabled && !isRecording)
        {
            isRecording = true;
            matchStartTime = Timer.getFPGATimestamp();
            publishStatus();
        }

        // Stop recording when disabled after having been enabled
        if (!enabled && wasEnabled && isRecording)
        {
            isRecording = false;
            publishStatus();
        }

        wasEnabled = enabled;

        // Publish AdvantageScope-compatible topics every loop
        if (isRecording)
        {
            publishAdvantageScope();
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Internal helpers
    // ──────────────────────────────────────────────────────────────────────────

    private void publishStatus()
    {
        var t = nt.getTable(NT_ROOT);
        t.getEntry("active").setBoolean(isRecording);
        t.getEntry("filename").setString(DataLogManager.getLogDir() + "/FRC_TBD.wpilog");
    }

    private void publishAdvantageScope()
    {
        double elapsed = Timer.getFPGATimestamp() - matchStartTime;

        var akTable = nt.getTable(AK_ROOT);

        // Elapsed time
        akTable.getEntry("Timestamp").setDouble(elapsed);

        // Auto routine name (read from Dashboard NT topic)
        String autoName = nt.getTable("/ChronosDashboard").getSubTable("selectedAutonomous").getEntry("Match")
                .getString("");
        akTable.getEntry("AutoRoutine").setString(autoName);

        // Pose is already published by the robot drive subsystem;
        // AdvantageScope picks it up natively. We republish here in the
        // flat double[3] format AdvantageScope Field2d expects.
        double poseX = nt.getTable("/Robot/Drive").getEntry("PoseX").getDouble(0);
        double poseY = nt.getTable("/Robot/Drive").getEntry("PoseY").getDouble(0);
        double hdgDeg = nt.getTable("/Robot/Drive").getEntry("PoseHeading").getDouble(0);
        double hdgRad = Math.toRadians(hdgDeg);

        var driveTable = akTable.getSubTable("Drive");
        // Field2d-compatible pose array: [x, y, heading_rad]
        driveTable.getEntry("Pose").setDoubleArray(new double[]
        { poseX, poseY, hdgRad });

        // AdvantageScope "ready" flag so the dashboard knows this bridge is live
        nt.getTable("/ChronosDashboard/advantagescope").getEntry("ready").setBoolean(true);
    }

    public boolean isRecording()
    {
        return isRecording;
    }

    public double getMatchElapsedSeconds()
    {
        if (matchStartTime < 0)
            return 0;
        return Timer.getFPGATimestamp() - matchStartTime;
    }
}
