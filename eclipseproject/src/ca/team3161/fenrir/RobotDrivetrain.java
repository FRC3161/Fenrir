package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.RepeatingIndependentSubsystem;
import ca.team3161.lib.utils.controls.Gamepad;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class RobotDrivetrain extends RepeatingIndependentSubsystem {

    private final Gamepad gamepad;
    private final Gyro gyro;
    private final RobotDrive driveBase;
    private final SpeedController frontLeft, frontRight, backLeft, backRight;
    private final Encoder frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    public RobotDrivetrain(final Gamepad gamepad,
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
        this.gamepad = gamepad;
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

    private void drive() {
        driveBase.mecanumDrive_Cartesian(
                gamepad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.X),
                gamepad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y),
                gamepad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.X),
                gyro.getAngle()
                );
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

}
