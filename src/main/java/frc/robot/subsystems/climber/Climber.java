package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.commands.HomeCommand;

public class Climber extends SubsystemBase {
    private final ClimberIO climber;
    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    public Command getHomingCommand() {
        return new HomeCommand(this);
    }

    public void homeDown() {
        climber.homeDown();
    }

    public boolean isAtBottom() {
        return climber.atBottom();
    }

    public void stopMotor() {
        climber.stopMotor();
    }
}
