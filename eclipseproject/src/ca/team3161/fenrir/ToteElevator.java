package ca.team3161.fenrir;

import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.RepeatingSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;


public class ToteElevator extends RepeatingSubsystem {

    private final SpeedController leftElevator, rightElevator, leftIntake, rightIntake,
                elevatorControllers, intakeControllers;
    private final Encoder leftEncoder, rightEncoder;
    private volatile Future<?> previousElevatorCommand, previousIntakeCommand;

    public ToteElevator(final SpeedController leftElevator, final SpeedController rightElevator,
            final SpeedController leftIntake, final SpeedController rightIntake,
            final Encoder leftEncoder, final Encoder rightEncoder) {
        super(50, TimeUnit.MILLISECONDS);
        this.leftElevator = leftElevator;
        this.rightElevator = rightElevator;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.elevatorControllers = new Drivetrain(leftElevator, rightElevator);
        this.intakeControllers = new Drivetrain(leftIntake, rightIntake);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    @Override
    public void defineResources() {
        require(leftElevator);
        require(rightElevator);
        require(leftIntake);
        require(rightIntake);
        require(rightEncoder);
        require(leftEncoder);
    }

    public void advanceElevator() {
        if (previousElevatorCommand != null) {
            previousElevatorCommand.cancel(true);
        }
        elevatorControllers.set(0.5);
        previousElevatorCommand = Executors.newSingleThreadExecutor().submit(() -> elevatorControllers.set(0));
    }

    public void retreatElevator() {
        if (previousElevatorCommand != null) {
            previousElevatorCommand.cancel(true);
        }
        elevatorControllers.set(-0.5);
        previousElevatorCommand = Executors.newSingleThreadExecutor().submit(() -> elevatorControllers.set(0));
    }

    public void startIntake() {
        if (previousIntakeCommand != null) {
            previousIntakeCommand.cancel(true);
        }
        intakeControllers.set(0.5);
        previousIntakeCommand = Executors.newSingleThreadExecutor().submit(() -> intakeControllers.set(0));
    }

    public void stopIntake() {
        if (previousIntakeCommand != null) {
            previousIntakeCommand.cancel(true);
        }
        intakeControllers.set(-0.5);
        previousIntakeCommand = Executors.newSingleThreadExecutor().submit(() -> intakeControllers.set(0));
    }

    @Override
    public void task() {
        // TODO: implement PIDs, use this to loop them
    }

}
