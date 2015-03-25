package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.pid.VelocityController;
import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ToteElevator extends RepeatingPooledSubsystem {

    private static final int TOTE_ENCODER_OVERSHOOT_TICKS = 100;
    private static final float ELEVATOR_RATE = 0.9f;
    public static final double INTAKE_MOTOR_PWM = 0.65;
    private final VelocityController leftElevator, rightElevator;
    private final SpeedController leftIntake, rightIntake, intakeControllers;
    private final Encoder leftEncoder, rightEncoder;
    private final Solenoid solenoid;
    private final DigitalInput leftPhotoGate, rightPhotoGate;
    private volatile int leftHookCountTarget = 0, rightHookCountTarget;
    private volatile int leftHookCount = 0, rightHookCount = 0, leftEncoderTicks = 0, rightEncoderTicks = 0;
    private volatile float leftPidTarget = 0, rightPidTarget = 0;

    public ToteElevator(final VelocityController leftElevator, final VelocityController rightElevator,
            final SpeedController leftIntake, final SpeedController rightIntake,
            final Encoder leftEncoder, final Encoder rightEncoder,
            final DigitalInput leftPhotoGate, final DigitalInput rightPhotoGate,
            final Solenoid solenoid) {
        super(5, TimeUnit.MILLISECONDS);
        this.leftElevator = leftElevator;
        this.rightElevator = rightElevator;
        this.leftIntake = leftIntake;
        this.rightIntake = rightIntake;
        this.intakeControllers = new Drivetrain(leftIntake, rightIntake);
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.leftPhotoGate = leftPhotoGate;
        this.rightPhotoGate = rightPhotoGate;
        this.solenoid = solenoid;
    }

    @Override
    public void defineResources() {
        require(leftElevator);
        require(rightElevator);
        require(leftIntake);
        require(rightIntake);
        require(solenoid);
    }

    public void setIntake(final double rate) {
        intakeControllers.set(rate);
    }

    public void stopIntake() {
        setIntake(0);
    }

    public void advanceElevatorCommand() {
        leftPidTarget = ELEVATOR_RATE;
        rightPidTarget = ELEVATOR_RATE;
        leftEncoderTicks = getLeftEncoder().get();
        rightEncoderTicks = getRightEncoder().get();
        leftHookCountTarget++;
        rightHookCountTarget++;
    }

    public void retreatElevatorCommand() {
        leftPidTarget = -ELEVATOR_RATE;
        rightPidTarget = -ELEVATOR_RATE;
        leftEncoderTicks = getLeftEncoder().get();
        rightEncoderTicks = getRightEncoder().get();
        leftHookCountTarget++;
        rightHookCountTarget++;
    }

    public void stopElevatorCommand() {
        stopLeftElevator();
        stopRightElevator();
    }

    public void stopLeftElevator() {
        leftPidTarget = 0;
    }

    public void stopRightElevator() {
        rightPidTarget = 0;
    }

    public void startIntakeCommand() {
        openClaws();
        setIntake(INTAKE_MOTOR_PWM);
    }

    public void stopIntakeCommand() {
        closeClaws();
        stopIntake();
    }

    public void openClaws() {
        solenoid.set(true);
    }

    public void closeClaws() {
        solenoid.set(false);
    }

    @Override
    public void task() {
        leftElevator.set(leftPidTarget);
        rightElevator.set(rightPidTarget);
        SmartDashboard.putNumber("left elev enc", getLeftEncoder().get());
        SmartDashboard.putNumber("right elev enc", getRightEncoder().get());
        if (!leftPhotoGate.get() && (leftEncoder.get() > leftEncoderTicks + TOTE_ENCODER_OVERSHOOT_TICKS || leftEncoder.get() < leftEncoderTicks - TOTE_ENCODER_OVERSHOOT_TICKS)) {
            leftHookCount++;
        }
        if (!rightPhotoGate.get() && (rightEncoder.get() > rightEncoderTicks + TOTE_ENCODER_OVERSHOOT_TICKS || rightEncoder.get() < rightEncoderTicks - TOTE_ENCODER_OVERSHOOT_TICKS)) {
            rightHookCount++;
        }
        if (leftHookCount >= leftHookCountTarget) {
            leftHookCount = leftHookCountTarget;
            stopLeftElevator();
        }
        if (rightHookCount >= rightHookCountTarget) {
            rightHookCount = rightHookCountTarget;
            stopRightElevator();
        }
    }

    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }

}
