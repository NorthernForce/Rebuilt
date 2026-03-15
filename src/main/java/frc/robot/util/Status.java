package frc.robot.util;

import java.util.function.Supplier;

public class Status
{
    private Supplier<String> message;
    private Status[] subStatuses;
    private String name;
    private Supplier<Boolean> isNotSevere;

    /**
     * Severity of overarching status of many substatuses (the elevator has motors,
     * for example, that all have statuses)
     * 
     * @param name        name of the system the status describes
     * @param subStatuses a list of substatuses that contribute to the overall
     *                    status
     * @implNote The overall status is severe if any substatus is severe, hence no
     *           isNotSevere supplier is needed, nor is a message
     */

    public Status(String name, Status... subStatuses)
    {
        this.subStatuses = subStatuses;
        this.name = name;
    }

    /**
     * Constructor for a status with no substatuses
     * 
     * @param name        name of the system the status describes
     * @param isNotSevere a boolean supplier that determines whether the status is
     *                    not severe or severe
     * @param message     a message describing the status
     */

    public Status(String name, Supplier<Boolean> isNotSevere, Supplier<String> message)
    {
        this.message = message;
        this.subStatuses = new Status[0];
        this.name = name;
        this.isNotSevere = isNotSevere;
    }

    /**
     * Gets all substatuses
     * 
     * @return a list of all substatuses
     */

    public Status[] getSubStatuses()
    {
        return subStatuses;
    }

    /**
     * Gets the overall severity of the status and its substatuses
     * 
     * @return the overall severity
     * @implNote If any substatus is severe, the overall status is severe.
     */

    public Status getConflictingStatus()
    {
        if (subStatuses != null && subStatuses.length > 0)
        {
            for (Status subStatus : subStatuses)
            {
                Status conflict = subStatus.getConflictingStatus();
                if (conflict != null)
                {
                    return conflict;
                }
            }
            return null;
        } else
        {
            return isNotSevere != null && isNotSevere.get() ? null : this;
        }
    }

    public Severity getConflictingSeverity()
    {
        Status conflictingStatus = getConflictingStatus();
        return conflictingStatus == null ? Severity.OK : Severity.ERROR;
    }

    /**
     * Gets the message of the status
     * 
     * @return the message
     */

    public String getConflictingMessage()
    {
        Status conflictingStatus = getConflictingStatus();
        return conflictingStatus == null ? null : conflictingStatus.getMessage();
    }

    /**
     * Gets the name of the conflicting status
     * 
     * @return the name of the conflicting status, or null if no conflict
     */

    public String getConflictingName()
    {
        Status conflictingStatus = getConflictingStatus();
        return conflictingStatus == null ? null : conflictingStatus.getName();
    }

    /**
     * Gets the name of the status
     * 
     * @return the name
     */

    public String getName()
    {
        return name;
    }

    /**
     * Get message of the status
     */

    public String getMessage()
    {
        return message != null ? message.get() : "No message available";
    }

    /**
     * Enum for severity levels
     */

    public enum Severity
    {
        OK, ERROR
    }
}
