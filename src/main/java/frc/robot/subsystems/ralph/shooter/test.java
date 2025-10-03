package frc.robot.subsystems.ralph.shooter;

public class test {
    ShooterIOTalonFX motor = new ShooterIOTalonFX(0);
    public test() {
        motor.outtake();
        motor.intake();
        motor.stop();
        
    }
}
