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

    private void setElevator(final double rate) {
        elevatorControllers.set(rate);
    }

    private void stopElevator() {
        setElevator(0);
    }

    private void setIntake(final double rate) {
        intakeControllers.set(rate);
    }

    private void stopIntake() {
        setIntake(0);
    }

    public void advanceElevatorCommand() {
        if (previousElevatorCommand != null) {
            previousElevatorCommand.cancel(true);
        }
        setElevator(0.5);
        previousElevatorCommand = Executors.newSingleThreadExecutor().submit(this::stopElevator);
    }

    public void retreatElevatorCommand() {
        if (previousElevatorCommand != null) {
            previousElevatorCommand.cancel(true);
        }
        setElevator(-0.5);
        previousElevatorCommand = Executors.newSingleThreadExecutor().submit(this::stopElevator);
    }

    public void startIntakeCommand() {
        if (previousIntakeCommand != null) {
            previousIntakeCommand.cancel(true);
        }
        setIntake(0.5);
        previousIntakeCommand = Executors.newSingleThreadExecutor().submit(this::stopIntake);
    }

    public void stopIntakeCommand() {
        if (previousIntakeCommand != null) {
            previousIntakeCommand.cancel(true);
        }
        setIntake(-0.5);
        previousIntakeCommand = Executors.newSingleThreadExecutor().submit(this::stopIntake);
    }

    @Override
    public void task() {
        // TODO: implement PIDs, use this to loop them
    }

}
