package ca.team3161.fenrir;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.SpeedController;

public class ToteElevator {
    
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final SpeedController elevatorControllers, intakeControllers;
    
    public ToteElevator(final SpeedController leftElevator, final SpeedController rightElevator,
            final SpeedController leftIntake, final SpeedController rightIntake) {
        this.elevatorControllers = new ca.team3161.lib.robot.Drivetrain(leftElevator, rightElevator);
        this.intakeControllers = new ca.team3161.lib.robot.Drivetrain(leftIntake, rightIntake);
    }
    
    private void set(final double rate) {
        elevatorControllers.set(rate);
    }
    
    private void waitAndStop() {
        try {
            Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
        } catch (final InterruptedException ie) {
            // don't care
        }
        set(0);
    }
    
    public void advance() {
        elevatorControllers.set(0.5);
        executor.submit(this::waitAndStop);
    }
    
    public void retreat() {
        elevatorControllers.set(-0.5);
        executor.submit(this::waitAndStop);
    }
    
    public void startIntake() {
        intakeControllers.set(0.3);
    }
    
    public void stopIntake() {
        intakeControllers.set(0);
    }

}
