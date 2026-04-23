package frc.robot.lobby.subsystems.nfrdashboard;

import java.util.Map;

import static edu.wpi.first.units.Units.Degrees;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase
{
    private String outputPath;

    private Map<String, Command> namesToCommands = new HashMap<>();
    private Map<String, Long> namesToLastRequestId = new HashMap<>();
    private Map<String, Command> namesToSafeCommands = new HashMap<>();
    private Map<String, Command> namesToKeybindOnFalseSafeCommands = new HashMap<>();
    private Map<String, Boolean> namesToLastKeybindPressed = new HashMap<>();
    private Map<String, DoubleSupplier> namesToDoubles = new HashMap<>();
    private Map<String, Supplier<String>> namesToStrings = new HashMap<>();
    private Map<String, BooleanSupplier> namesToBooleans = new HashMap<>();
    private Map<String, Consumer<Double>> namesToDoubleTunables = new HashMap<>();
    private Map<String, Consumer<String>> namesToStringsTunables = new HashMap<>();
    private Map<String, Consumer<Boolean>> namesToBooleansTunables = new HashMap<>();
    private Map<String, DashboardField> namesToFields = new HashMap<>();
    private Map<String, DashboardRobot> namesToRobots = new HashMap<>();

    private NetworkTableInstance instance = NetworkTableInstance.getDefault();

    // public Dashboard(String outputPath) {
    // this.outputPath = outputPath;
    // }

    public Dashboard()
    {
        outputPath = "/NFRDashboard";
    }

    private String makeKey(String tab, String table, String section, String name)
    {
        return tab + "::" + table + "::" + section + "::" + name;
    }

    private static class ParsedKey
    {
        final String tab, table, section, name;

        ParsedKey(String tab, String table, String section, String name)
        {
            this.tab = tab;
            this.table = table;
            this.section = section;
            this.name = name;
        }
    }

    private ParsedKey parseKey(String key)
    {
        String[] parts = key.split("::", 4);
        if (parts.length != 4)
            return null;
        return new ParsedKey(parts[0], parts[1], parts[2], parts[3]);
    }

    private NetworkTable scopedEntry(String table, String section, String tab, String name)
    {
        return instance.getTable(table).getSubTable(section).getSubTable(tab).getSubTable(name);
    }

    private NetworkTable robotEntry(String tab, String field, String robotName)
    {
        return instance.getTable(outputPath).getSubTable("robots").getSubTable(tab).getSubTable(field)
                .getSubTable(robotName);
    }

    public void putKeybind(char key, String description, Command command)
    {
        putKeybind(key, description, command, null);
    }

    public void putKeybind(char key, String description, Command onTrue, Command onFalse)
    {
        String keyStr = Character.toString(key);
        String keybindKey = makeKey("Keybinds", outputPath, "commands", keyStr);
        if (!namesToCommands.containsKey(keybindKey))
        {
            namesToCommands.put(keybindKey, onTrue);
            namesToLastRequestId.put(keybindKey, 0L);
            namesToLastKeybindPressed.put(keybindKey, false);

            var commandTable = scopedEntry(outputPath, "commands", "Keybinds", keyStr);
            commandTable.getEntry("running").setBoolean(false);
            commandTable.getEntry("requested").setBoolean(false);
            commandTable.getEntry("requestId").setInteger(0);
            commandTable.getEntry("lastHandledRequestId").setInteger(0);
            commandTable.getEntry("pressed").setBoolean(false);
            commandTable.getEntry("tab").setString("Keybinds");
            commandTable.getEntry("description").setString(description);

            Command safeOnTrue = new Command()
            {
                final Command inner = onTrue;

                @Override
                public void initialize()
                {
                    var t = scopedEntry(outputPath, "commands", "Keybinds", keyStr);
                    t.getEntry("running").setBoolean(true);
                    CommandScheduler.getInstance().schedule(inner);
                }

                @Override
                public void execute()
                {
                }

                @Override
                public void end(boolean interrupted)
                {
                    if (CommandScheduler.getInstance().isScheduled(inner))
                        CommandScheduler.getInstance().cancel(inner);
                    scopedEntry(outputPath, "commands", "Keybinds", keyStr).getEntry("running").setBoolean(false);
                }

                @Override
                public boolean isFinished()
                {
                    return !CommandScheduler.getInstance().isScheduled(inner);
                }
            };

            namesToSafeCommands.put(keybindKey, safeOnTrue);

            if (onFalse != null)
            {
                Command safeOnFalse = new Command()
                {
                    final Command inner = onFalse;

                    @Override
                    public void initialize()
                    {
                        var t = scopedEntry(outputPath, "commands", "Keybinds", keyStr);
                        t.getEntry("running").setBoolean(true);
                        CommandScheduler.getInstance().schedule(inner);
                    }

                    @Override
                    public void execute()
                    {
                    }

                    @Override
                    public void end(boolean interrupted)
                    {
                        if (CommandScheduler.getInstance().isScheduled(inner))
                            CommandScheduler.getInstance().cancel(inner);
                        scopedEntry(outputPath, "commands", "Keybinds", keyStr).getEntry("running").setBoolean(false);

                    }

                    @Override
                    public boolean isFinished()
                    {
                        return !CommandScheduler.getInstance().isScheduled(inner);
                    }
                };
                namesToKeybindOnFalseSafeCommands.put(keybindKey, safeOnFalse);
            }
        }
    }

    public boolean putCommand(String tab, String name, Command command)
    {
        String key = makeKey(tab, outputPath, "commands", name);
        if (!namesToCommands.containsKey(key))
        {
            namesToCommands.put(key, command);
            namesToLastRequestId.put(key, 0L);

            var commandTable = scopedEntry(outputPath, "commands", tab, name);
            commandTable.getEntry("running").setBoolean(false);
            commandTable.getEntry("requested").setBoolean(false);
            commandTable.getEntry("requestId").setInteger(0);
            commandTable.getEntry("lastHandledRequestId").setInteger(0);
            commandTable.getEntry("tab").setString(tab);

            Command safeCommand = new Command()
            {
                final Command inner = command;

                @Override
                public void initialize()
                {
                    var t = scopedEntry(outputPath, "commands", tab, name);
                    t.getEntry("requested").setBoolean(false);
                    t.getEntry("running").setBoolean(true);
                    CommandScheduler.getInstance().schedule(inner);
                }

                @Override
                public void execute()
                {
                }

                @Override
                public void end(boolean interrupted)
                {
                    if (CommandScheduler.getInstance().isScheduled(inner))
                        CommandScheduler.getInstance().cancel(inner);
                    scopedEntry(outputPath, "commands", tab, name).getEntry("running").setBoolean(false);
                }

                @Override
                public boolean isFinished()
                {
                    return !CommandScheduler.getInstance().isScheduled(inner);
                }
            };

            namesToSafeCommands.put(key, safeCommand);

            return true;
        }
        return false;
    }

    public boolean putCommand(String tab, String table, String name, Command command)
    {
        String key = makeKey(tab, table, "commands", name);
        if (!namesToCommands.containsKey(key))
        {
            namesToCommands.put(key, command);
            namesToLastRequestId.put(key, 0L);

            var commandTable = scopedEntry(table, "commands", tab, name);
            commandTable.getEntry("running").setBoolean(false);
            commandTable.getEntry("requested").setBoolean(false);
            commandTable.getEntry("requestId").setInteger(0);
            commandTable.getEntry("lastHandledRequestId").setInteger(0);
            commandTable.getEntry("tab").setString(tab);

            Command safeCommand = new Command()
            {
                final Command inner = command;

                @Override
                public void initialize()
                {
                    var t = scopedEntry(table, "commands", tab, name);
                    t.getEntry("requested").setBoolean(false);
                    t.getEntry("running").setBoolean(true);
                    CommandScheduler.getInstance().schedule(inner);
                }

                @Override
                public void execute()
                {
                }

                @Override
                public void end(boolean interrupted)
                {
                    if (CommandScheduler.getInstance().isScheduled(inner))
                        CommandScheduler.getInstance().cancel(inner);
                    scopedEntry(table, "commands", tab, name).getEntry("running").setBoolean(false);
                }

                @Override
                public boolean isFinished()
                {
                    return !CommandScheduler.getInstance().isScheduled(inner);
                }
            };

            namesToSafeCommands.put(key, safeCommand);

            return true;
        }
        return false;
    }

    public void putLimelightStream(String tab, String llName)
    {
        var streamTable = scopedEntry(outputPath, "cameraStreams", tab, llName);
        streamTable.getEntry("url").setString("http://" + llName + ".local:5800");
        streamTable.getEntry("tab").setString(tab);
        streamTable.getEntry("name").setString(llName);
    }

    public void putCameraStream(String tab, String name, String url)
    {
        var streamTable = scopedEntry(outputPath, "cameraStreams", tab, name);
        streamTable.getEntry("url").setString(url);
        streamTable.getEntry("tab").setString(tab);
        streamTable.getEntry("name").setString(name);
    }

    public void putNumber(String tab, String name, DoubleSupplier number)
    {
        String key = makeKey(tab, outputPath, "numbers", name);
        if (!namesToDoubles.containsKey(key))
        {
            namesToDoubles.put(key, number);
            var t = scopedEntry(outputPath, "numbers", tab, name);
            t.getEntry("value").setDouble(number.getAsDouble());
            t.getEntry("tab").setString(tab);
        }
    }

    public void putNumber(String tab, String table, String name, DoubleSupplier number)
    {
        String key = makeKey(tab, table, "numbers", name);
        if (!namesToDoubles.containsKey(key))
        {
            namesToDoubles.put(key, number);
            var t = scopedEntry(table, "numbers", tab, name);
            t.getEntry("value").setDouble(number.getAsDouble());
            t.getEntry("tab").setString(tab);
        }
    }

    public void putString(String tab, String name, Supplier<String> string)
    {
        String key = makeKey(tab, outputPath, "strings", name);
        if (!namesToStrings.containsKey(key))
        {
            namesToStrings.put(key, string);
            var t = scopedEntry(outputPath, "strings", tab, name);
            t.getEntry("value").setString(string.get());
            t.getEntry("tab").setString(tab);
        }
    }

    public void putString(String tab, String table, String name, Supplier<String> string)
    {
        String key = makeKey(tab, table, "strings", name);
        if (!namesToStrings.containsKey(key))
        {
            namesToStrings.put(key, string);
            var t = scopedEntry(table, "strings", tab, name);
            t.getEntry("value").setString(string.get());
            t.getEntry("tab").setString(tab);
        }
    }

    public void putBoolean(String tab, String name, BooleanSupplier booleanSupplier)
    {
        String key = makeKey(tab, outputPath, "booleans", name);
        if (!namesToBooleans.containsKey(key))
        {
            namesToBooleans.put(key, booleanSupplier);
            var t = scopedEntry(outputPath, "booleans", tab, name);
            t.getEntry("value").setBoolean(booleanSupplier.getAsBoolean());
            t.getEntry("tab").setString(tab);
        }
    }

    public void putBoolean(String tab, String table, String name, BooleanSupplier booleanSupplier)
    {
        String key = makeKey(tab, table, "booleans", name);
        if (!namesToBooleans.containsKey(key))
        {
            namesToBooleans.put(key, booleanSupplier);
            var t = scopedEntry(table, "booleans", tab, name);
            t.getEntry("value").setBoolean(booleanSupplier.getAsBoolean());
            t.getEntry("tab").setString(tab);
        }
    }

    public void putNumberTunable(String tab, String name, Consumer<Double> runOnChange)
    {
        String key = makeKey(tab, outputPath, "tunableNumbers", name);
        if (!namesToDoubleTunables.containsKey(key))
        {
            namesToDoubleTunables.put(key, runOnChange);
            var t = scopedEntry(outputPath, "tunableNumbers", tab, name);
            t.getEntry("value").setNumber(0);
            t.getEntry("changed").setBoolean(false);
            t.getEntry("tab").setString(tab);
        }
    }

    public void putNumberTunable(String tab, String table, String name, Consumer<Double> runOnChange)
    {
        String key = makeKey(tab, table, "tunableNumbers", name);
        if (!namesToDoubleTunables.containsKey(key))
        {
            namesToDoubleTunables.put(key, runOnChange);
            var t = scopedEntry(table, "tunableNumbers", tab, name);
            t.getEntry("value").setNumber(0);
            t.getEntry("changed").setBoolean(false);
            t.getEntry("tab").setString(tab);
        }
    }

    public void putStringTunable(String tab, String name, Consumer<String> runOnChange)
    {
        String key = makeKey(tab, outputPath, "tunableStrings", name);
        if (!namesToStringsTunables.containsKey(key))
        {
            namesToStringsTunables.put(key, runOnChange);
            var t = scopedEntry(outputPath, "tunableStrings", tab, name);
            t.getEntry("value").setString("");
            t.getEntry("changed").setBoolean(false);
            t.getEntry("tab").setString(tab);
        }
    }

    public void putStringTunable(String tab, String table, String name, Consumer<String> runOnChange)
    {
        String key = makeKey(tab, table, "tunableStrings", name);
        if (!namesToStringsTunables.containsKey(key))
        {
            namesToStringsTunables.put(key, runOnChange);
            var t = scopedEntry(table, "tunableStrings", tab, name);
            t.getEntry("value").setString("");
            t.getEntry("changed").setBoolean(false);
            t.getEntry("tab").setString(tab);
        }
    }

    public void putBooleanTunable(String tab, String name, Consumer<Boolean> runOnChange)
    {
        String key = makeKey(tab, outputPath, "tunableBooleans", name);
        if (!namesToBooleansTunables.containsKey(key))
        {
            namesToBooleansTunables.put(key, runOnChange);
            var t = scopedEntry(outputPath, "tunableBooleans", tab, name);
            t.getEntry("value").setBoolean(false);
            t.getEntry("changed").setBoolean(false);
            t.getEntry("tab").setString(tab);
        }
    }

    public void putBooleanTunable(String tab, String table, String name, Consumer<Boolean> runOnChange)
    {
        String key = makeKey(tab, table, "tunableBooleans", name);
        if (!namesToBooleansTunables.containsKey(key))
        {
            namesToBooleansTunables.put(key, runOnChange);
            var t = scopedEntry(table, "tunableBooleans", tab, name);
            t.getEntry("value").setBoolean(false);
            t.getEntry("changed").setBoolean(false);
            t.getEntry("tab").setString(tab);
        }
    }

    public DashboardField putField(String tab, String name)
    {
        String key = makeKey(tab, outputPath, "fields", name);
        if (!namesToFields.containsKey(key))
        {
            DashboardField field = new DashboardField(tab, name);
            namesToFields.put(key, field);
            var t = scopedEntry(outputPath, "fields", tab, name);
            t.getEntry("tab").setString(tab);
            t.getEntry("name").setString(name);
            return field;
        }
        return namesToFields.get(key);
    }

    public DashboardRobot putRobot(String tab, String field, String name, Supplier<Pose2d> pose)
    {
        // ensure relationship target exists
        putField(tab, field);

        String key = makeKey(tab, outputPath, "robots", field + "/" + name);
        if (!namesToRobots.containsKey(key))
        {
            DashboardRobot robot = new DashboardRobot(tab, field, name, pose);
            namesToRobots.put(key, robot);

            var robotTable = robotEntry(tab, field, name);
            robotTable.getEntry("tab").setString(tab);
            robotTable.getEntry("field").setString(field);
            robotTable.getEntry("name").setString(name);
            robotTable.getEntry("x").setDouble(pose.get().getX());
            robotTable.getEntry("y").setDouble(pose.get().getY());
            robotTable.getEntry("rotation").setDouble(pose.get().getRotation().getMeasure().in(Degrees));
            return robot;
        }
        return namesToRobots.get(key);
    }

    @Override
    public void periodic()
    {
        for (String key : namesToCommands.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = scopedEntry(p.table, "commands", p.tab, p.name);
            long currentRequestId = table.getEntry("requestId").getInteger(0);
            long lastHandledRequestId = namesToLastRequestId.getOrDefault(key, 0L);
            if (currentRequestId != lastHandledRequestId)
            {
                namesToLastRequestId.put(key, currentRequestId);
                table.getEntry("lastHandledRequestId").setInteger(currentRequestId);
                table.getEntry("requested").setBoolean(false);
                Command safe = namesToSafeCommands.get(key);
                if (safe != null)
                {
                    if (CommandScheduler.getInstance().isScheduled(safe))
                        CommandScheduler.getInstance().cancel(safe);
                    else
                        CommandScheduler.getInstance().schedule(safe);
                }
            }
        }

        for (String key : namesToLastKeybindPressed.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null || !"Keybinds".equals(p.tab))
                continue;

            var table = scopedEntry(p.table, "commands", p.tab, p.name);
            boolean pressed = table.getEntry("pressed").getBoolean(false);
            boolean lastPressed = namesToLastKeybindPressed.getOrDefault(key, false);

            if (pressed != lastPressed)
            {
                namesToLastKeybindPressed.put(key, pressed);
                if (pressed)
                {
                    Command safeOnTrue = namesToSafeCommands.get(key);
                    if (safeOnTrue != null)
                    {
                        if (CommandScheduler.getInstance().isScheduled(safeOnTrue))
                            CommandScheduler.getInstance().cancel(safeOnTrue);
                        CommandScheduler.getInstance().schedule(safeOnTrue);
                    }
                } else
                {
                    Command safeOnFalse = namesToKeybindOnFalseSafeCommands.get(key);
                    if (safeOnFalse != null)
                    {
                        if (CommandScheduler.getInstance().isScheduled(safeOnFalse))
                            CommandScheduler.getInstance().cancel(safeOnFalse);
                        CommandScheduler.getInstance().schedule(safeOnFalse);
                    }
                    table.getEntry("running").setBoolean(false);
                }
            }
        }

        for (String key : namesToDoubles.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            scopedEntry(p.table, "numbers", p.tab, p.name).getEntry("value")
                    .setDouble(namesToDoubles.get(key).getAsDouble());
        }
        for (String key : namesToStrings.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            scopedEntry(p.table, "strings", p.tab, p.name).getEntry("value").setString(namesToStrings.get(key).get());
        }
        for (String key : namesToBooleans.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            scopedEntry(p.table, "booleans", p.tab, p.name).getEntry("value")
                    .setBoolean(namesToBooleans.get(key).getAsBoolean());
        }
        for (String key : namesToDoubleTunables.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = scopedEntry(p.table, "tunableNumbers", p.tab, p.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToDoubleTunables.get(key).accept(table.getEntry("value").getDouble(0));
                table.getEntry("changed").setBoolean(false);
            }
        }
        for (String key : namesToStringsTunables.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = scopedEntry(p.table, "tunableStrings", p.tab, p.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToStringsTunables.get(key).accept(table.getEntry("value").getString(""));
                table.getEntry("changed").setBoolean(false);
            }
        }
        for (String key : namesToBooleansTunables.keySet())
        {
            ParsedKey p = parseKey(key);
            if (p == null)
                continue;
            var table = scopedEntry(p.table, "tunableBooleans", p.tab, p.name);
            if (table.getEntry("changed").getBoolean(false))
            {
                namesToBooleansTunables.get(key).accept(table.getEntry("value").getBoolean(false));
                table.getEntry("changed").setBoolean(false);
            }
        }

        for (String key : namesToRobots.keySet())
        {
            DashboardRobot robot = namesToRobots.get(key);
            var robotTable = robotEntry(robot.tab, robot.fieldName, robot.robotName);
            Pose2d pose = robot.pose.get();
            robotTable.getEntry("x").setDouble(pose.getX());
            robotTable.getEntry("y").setDouble(pose.getY());
            robotTable.getEntry("rotation").setDouble(pose.getRotation().getMeasure().in(Degrees));
        }
    }

    public class DashboardRobot
    {
        private Supplier<Pose2d> pose;
        private String tab;
        private String fieldName;
        private String robotName;

        public DashboardRobot(String tab, String fieldName, String robotName, Supplier<Pose2d> pose)
        {
            this.pose = pose;
            this.tab = tab;
            this.fieldName = fieldName;
            this.robotName = robotName;
        }
    }

    public class DashboardField
    {
        private String tab;
        private String fieldName;

        public DashboardField(String tab, String name)
        {
            this.tab = tab;
            this.fieldName = name;
        }

        public DashboardField withRobot(String name, Supplier<Pose2d> pose)
        {
            Dashboard.this.putRobot(tab, fieldName, name, pose);
            return this;
        }
    }

    public DashboardSystem putSystem(String tab, String systemName)
    {
        return new DashboardSystem(systemName, tab);
    }

    public class DashboardSystem
    {
        private String name;
        private String tab;

        public DashboardSystem(String name, String tab)
        {
            this.name = name;
            this.tab = tab;
        }

        public String getName()
        {
            return name;
        }

        public String getTab()
        {
            return tab;
        }

        public DashboardSystem withCommand(String name, Command command)
        {
            Dashboard.this.putCommand(tab, outputPath + "/systems/" + this.name, name, command);
            return this;
        }

        public DashboardSystem withNumber(String name, DoubleSupplier number)
        {
            Dashboard.this.putNumber(tab, outputPath + "/systems/" + this.name, name, number);
            return this;
        }

        public DashboardSystem withString(String name, Supplier<String> string)
        {
            Dashboard.this.putString(tab, outputPath + "/systems/" + this.name, name, string);
            return this;
        }

        public DashboardSystem withBoolean(String name, BooleanSupplier booleanSupplier)
        {
            Dashboard.this.putBoolean(tab, outputPath + "/systems/" + this.name, name, booleanSupplier);
            return this;
        }

        public DashboardSystem withNumberTunable(String name, Consumer<Double> runOnChange)
        {
            Dashboard.this.putNumberTunable(tab, outputPath + "/systems/" + this.name, name, runOnChange);
            return this;
        }

        public DashboardSystem withStringTunable(String name, Consumer<String> runOnChange)
        {
            Dashboard.this.putStringTunable(tab, outputPath + "/systems/" + this.name, name, runOnChange);
            return this;
        }

        public DashboardSystem withBooleanTunable(String name, Consumer<Boolean> runOnChange)
        {
            Dashboard.this.putBooleanTunable(tab, outputPath + "/systems/" + this.name, name, runOnChange);
            return this;

        }
    }
}
