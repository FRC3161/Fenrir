package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.pid.VelocityController;
import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ToteElevator extends RepeatingIndependentSubsystem {

    private static final double INTAKE_MOTOR_PWM = 0.3;
    private final VelocityController leftElevator, rightElevator;
	private final SpeedController leftIntake, rightIntake, intakeControllers;
    private final Encoder leftEncoder, rightEncoder;
    private final Solenoid solenoid;
    private volatile float pidTarget = 0;

    public ToteElevator(final VelocityController leftElevator, final VelocityController rightElevator,
            final SpeedController leftIntake, final SpeedController rightIntake,
            final Encoder leftEncoder, final Encoder rightEncoder,
            final Solenoid solenoid) {
        super(20, TimeUnit.MILLISECONDS);
        this.leftElevator = leftElevator;
        this.rightElevator = rightElevator;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
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

    private void setIntake(final double rate) {
        intakeControllers.set(rate);
    }

    private void stopIntake() {
        setIntake(0);
    }

    public void advanceElevatorCommand() {
        pidTarget = 0.5f;
    }

    public void retreatElevatorCommand() {
        pidTarget = -0.5f;
    }

    public void stopElevatorCommand() {
        pidTarget = 0;
    }

    public void startIntakeCommand() {
    	openClaws();
        setIntake(INTAKE_MOTOR_PWM);
    }

    public void stopIntakeCommand() {
    	closeClaws();
        stopIntake();
    }

    private void openClaws() {
        solenoid.set(false);
    }

    private void closeClaws() {
        solenoid.set(true);
    }

    @Override
    public void task() {
    	final float leftPid = leftElevator.pid(pidTarget * Robot.MAX_ELEVATOR_RATE);
    	final float rightPid = rightElevator.pid(pidTarget * Robot.MAX_ELEVATOR_RATE);
    	SmartDashboard.putNumber("Left PID", leftPid);
    	SmartDashboard.putNumber("Right PID", rightPid);
    	leftElevator.getSpeedController().set(leftPid);
    	rightElevator.getSpeedController().set(rightPid);
    	SmartDashboard.putNumber("left error", (pidTarget * Robot.MAX_ELEVATOR_RATE) - leftElevator.getRate());
    	
    	SmartDashboard.putNumber("Elev PID target", pidTarget);
    }
    
    public Encoder getLeftEncoder() {
    	return leftEncoder;
    }
    
    public Encoder getRightEncoder() {
    	return rightEncoder;
    }

}
