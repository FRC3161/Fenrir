package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.RepeatingIndependentSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;


public class ToteElevator extends RepeatingIndependentSubsystem {

    private static final double INTAKE_MOTOR_PWM = 0.3;
	private final SpeedController leftElevator, rightElevator, leftIntake, rightIntake,
                elevatorControllers, intakeControllers;
    private final Encoder leftEncoder, rightEncoder;
    private final Solenoid solenoid;

    public ToteElevator(final SpeedController leftElevator, final SpeedController rightElevator,
            final SpeedController leftIntake, final SpeedController rightIntake,
            final Encoder leftEncoder, final Encoder rightEncoder,
            final Solenoid solenoid) {
        super(50, TimeUnit.MILLISECONDS);
        this.leftElevator = leftElevator;
        this.rightElevator = rightElevator;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.elevatorControllers = new Drivetrain(leftElevator, rightElevator);
        this.intakeControllers = new Drivetrain(leftIntake, rightIntake);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.solenoid = solenoid;
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

    private void waitAndStopElevator() {
        try {
            Thread.sleep(500);
        } catch (final InterruptedException ie) {
        }
        stopElevator();
    }

    public void advanceElevatorCommand() {
        setElevator(INTAKE_MOTOR_PWM);
    }

    public void retreatElevatorCommand() {
        setElevator(-INTAKE_MOTOR_PWM);
    }

    public void stopElevatorCommand() {
        stopElevator();
    }

    public void startIntakeCommand() {
        setIntake(INTAKE_MOTOR_PWM);
    }

    public void stopIntakeCommand() {
        stopIntake();
    }

    public void openClawsCommand() {
        solenoid.set(false);
    }

    public void closeClawsCommand() {
        solenoid.set(true);
    }

    @Override
    public void task() {
        // TODO: implement PIDs, use this to loop them
    }

}
