package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ralph.RalphConstants.InnerElevatorConstants;
import frc.robot.ralph.RalphConstants.OuterElevatorConstants;
import frc.robot.subsystems.superstructure.elevator.Elevator;

public class Superstructure extends SubsystemBase
{
    private Elevator innerElevator;
    private Elevator outerElevator;
    private SuperstructureState state = SuperstructureState.START;
    private boolean stateChanged = true;
    private SuperstructurePosition goal;

    public Superstructure(Elevator innerElevator, Elevator outerElevator)
    {
        this.innerElevator = innerElevator;
        this.outerElevator = outerElevator;
    }

    public Superstructure()
    {
        innerElevator = new Elevator(InnerElevatorConstants.kCanID, InnerElevatorConstants.kSensorID,
                InnerElevatorConstants.kConfig);
        outerElevator = new Elevator(OuterElevatorConstants.kCanID, OuterElevatorConstants.kSensorID,
                OuterElevatorConstants.kConfig);
    }

    @Override
    public void periodic()
    {
        switch (state)
        {
        case L1, L2, L3, L4, CORAL_STATION, START:
            if (stateChanged)
            {
                setGoalPosition(state.position);
                stateChanged = false;
            }
            if (isAtGoal())
            {
                setState(SuperstructureState.HOLDING);
            }
        case HOLDING:
            if (stateChanged)
            {
                stateChanged = false;
                stop();
            }
        case MANUAL:
            if (stateChanged)
            {
                stateChanged = false;
            }
        }
    }

    public void stop()
    {
        innerElevator.stop();
        outerElevator.stop();
    }

    public class ManualControlCommand extends ParallelCommandGroup
    {
        public ManualControlCommand(Supplier<Double> innerElevatorSpeed, Supplier<Double> outerElevatorSpeed)
        {
            addCommands(Commands.run(() -> innerElevator.manualControl(innerElevatorSpeed.get()), innerElevator),
                    Commands.run(() -> outerElevator.manualControl(outerElevatorSpeed.get()), outerElevator));
        }
    }

    public SuperstructurePosition getPosition()
    {
        return new SuperstructurePosition(innerElevator.getPosition(), outerElevator.getPosition());
    }

    public void setState(SuperstructureState state)
    {
        this.state = state;
        stateChanged = true;
    }

    public void setGoalPosition(SuperstructurePosition goal)
    {
        this.goal = goal;
        innerElevator.setGoalPosition(goal.innerPosition());
        outerElevator.setGoalPosition(goal.outerPosition());
    }

    public boolean isAtGoal()
    {
        return innerElevator.isAtPosition(goal.innerPosition) && outerElevator.isAtPosition(goal.outerPosition);
    }

    public boolean isAtPosition(SuperstructurePosition position)
    {
        return innerElevator.isAtPosition(position.innerPosition) && outerElevator.isAtPosition(position.outerPosition);
    }

    public Elevator getInnerElevator()
    {
        return innerElevator;
    }

    public Elevator getOuterElevator()
    {
        return outerElevator;
    }

    public enum SuperstructureState
    {
        START(Inches.of(0), Inches.of(0)), L1(Inches.of(0), Inches.of(2)), L2(Inches.of(0), Inches.of(11.38)),
        L3(Inches.of(0), Inches.of(26.8)), L4(Inches.of(24.8), Inches.of(26.8)),
        CORAL_STATION(Inches.of(0), Inches.of(0)), HOLDING, MANUAL;

        private final SuperstructurePosition position;

        private SuperstructureState()
        {
            this.position = null;
        }

        private SuperstructureState(Distance innerPosition, Distance outerPosition)
        {
            this.position = new SuperstructurePosition(innerPosition, outerPosition);
        }
    }

    public record SuperstructurePosition(Distance innerPosition, Distance outerPosition) {
    }
}
