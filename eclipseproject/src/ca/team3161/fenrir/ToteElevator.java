package ca.team3161.fenrir;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.RepeatingSubsystem;
import edu.wpi.first.wpilibj.SpeedController;

public class ToteElevator extends RepeatingSubsystem {

    private final SpeedController leftElevator, rightElevator, leftIntake, rightIntake,
                elevatorControllers, intakeControllers;
    private final Queue<Runnable> commandQueue = new ConcurrentLinkedQueue<>();

    public ToteElevator(final SpeedController leftElevator, final SpeedController rightElevator,
            final SpeedController leftIntake, final SpeedController rightIntake) {
        super(50, TimeUnit.MILLISECONDS);
        this.leftElevator = leftElevator;
        this.rightElevator = rightElevator;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.elevatorControllers = new Drivetrain(leftElevator, rightElevator);
        this.intakeControllers = new Drivetrain(leftIntake, rightIntake);
    }

    @Override
    public void defineResources() {
        require(leftElevator);
        require(rightElevator);
        require(leftIntake);
        require(rightIntake);
    }

    private void setElevator(final double rate) {
        elevatorControllers.set(rate);
    }

    private void setIntake(final double rate) {
        intakeControllers.set(rate);
    }

    public void advanceElevator() {
        commandQueue.offer(() -> {
            setElevator(0.5);
            try {
                Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
            } catch (final InterruptedException ie) {
                // don't care
            }
            setElevator(0);
        });
    }

    public void retreatElevator() {
        commandQueue.offer(() -> {
            setElevator(-0.5);
            try {
                Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
            } catch (final InterruptedException ie) {
                // don't care
            }
            setElevator(0);
        });
    }

    public void startIntake() {
        commandQueue.offer(() -> {
            setIntake(0.5);
            try {
                Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
            } catch (final InterruptedException ie) {
                // don't care
            }
            setIntake(0);
        });
    }

    public void stopIntake() {
        commandQueue.offer(() -> {
            setIntake(-0.5);
            try {
                Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
            } catch (final InterruptedException ie) {
                // don't care
            }
            setIntake(0);
        });
    }

    @Override
    public void task() {
        commandQueue.forEach(Runnable::run);
    }

}
