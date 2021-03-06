package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class RobotDrivetrain extends RepeatingPooledSubsystem {

    private static final double TURBO_MODIFIER = 1.0;
    private static final double NONTURBO_MODIFIER = 0.2;
    private final Gyro gyro;
    private final RobotDrive driveBase;
    private final SpeedController frontLeft, frontRight, backLeft, backRight;
    private final Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private volatile boolean auton = true;
    private volatile double leftAutoTargetRate;
    private volatile double rightAutoTargetRate;
    private volatile double driveForward;
    private volatile double driveStrafe;
    private volatile double driveTurn;
    private volatile boolean turbo = false;

    public RobotDrivetrain(
            final SpeedController frontLeft, final SpeedController frontRight,
            final SpeedController backLeft, final SpeedController backRight,
            final Encoder frontLeftEncoder, final Encoder frontRightEncoder,
            final Encoder backLeftEncoder, final Encoder backRightEncoder,
            final Gyro gyro) {
        super(10, TimeUnit.MILLISECONDS);
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.driveBase = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
        this.frontLeftEncoder = frontLeftEncoder;
        this.frontRightEncoder = frontRightEncoder;
        this.backLeftEncoder = backLeftEncoder;
        this.backRightEncoder = backRightEncoder;
    }

    @Override
    public void defineResources() {
        require(gyro);
        require(frontLeft);
        require(frontRight);
        require(backLeft);
        require(backRight);
        require(frontLeftEncoder);
        require(frontRightEncoder);
        require(backLeftEncoder);
        require(backRightEncoder);
    }

    public void setAutonomous(final boolean auton) {
        this.auton = auton;
        if (!auton) {
            setAutoRates(0, 0);
        }
    }

    public void setAutoRates(final double left, final double right) {
        this.leftAutoTargetRate = left;
        this.rightAutoTargetRate = right;
    }

    public void enableTurbo() {
        this.turbo = true;
    }

    public void disableTurbo() {
        this.turbo = false;
    }

    public void setDriveForward(final double driveForward) {
        this.driveForward = driveForward;
    }

    public void setDriveStrafe(final double driveStrafe) {
        this.driveStrafe = driveStrafe;
    }

    public void setDriveTurn(final double driveTurn) {
        this.driveTurn = driveTurn;
    }

    private void drive() {
        if (auton) {
            driveBase.tankDrive(leftAutoTargetRate, rightAutoTargetRate);
        } else {
            final double adjustment = turbo ? TURBO_MODIFIER : NONTURBO_MODIFIER;
            driveBase.mecanumDrive_Cartesian(
                    driveStrafe * adjustment * 1.5,
                    driveForward * adjustment,
                    driveTurn * adjustment,
//                    gyro.getAngle()
                    0
                    );
        }
    }

    @Override
    public void task() {
        drive();
    }

    public Encoder getFLEncoder () {
        return frontLeftEncoder;
    }

    public Encoder getFREncoder () {
        return frontRightEncoder;
    }

    public Encoder getBLEncoder () {
        return backLeftEncoder;
    }

    public Encoder getBREncoder () {
        return backRightEncoder;
    }

    public SpeedController getFLController() {
        return frontLeft;
    }

    public SpeedController getFRController() {
        return frontRight;
    }

    public SpeedController getBLController() {
        return backLeft;
    }

    public SpeedController getBRController() {
        return backRight;
    }

}
