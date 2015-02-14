package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.RepeatingSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;


public class ToteElevator extends RepeatingSubsystem {

    private final SpeedController leftElevator, rightElevator, leftIntake, rightIntake,
                elevatorControllers, intakeControllers;
    private final Encoder leftEncoder, rightEncoder;
    private volatile double elevatorRate, intakeRate;

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
    }

    public void retreatElevator() {
    }

    public void startIntake() {
    }

    public void stopIntake() {
    }

    @Override
    public void task() {
    }

}
