package ca.team3161.fenrir;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class RobotDrivetrain {
    
    private final Gyro gyro;
    private final RobotDrive driveBase;
    
    public RobotDrivetrain(final SpeedController frontLeft, final SpeedController frontRight,
            final SpeedController backLeft, final SpeedController backRight,
            final Gyro gyro) {
        this.gyro = gyro;
        this.driveBase = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
    }
    
    public void drive(final double strafe, final double drive, final double rotation) {
        driveBase.mecanumDrive_Cartesian(strafe, drive, rotation, gyro.getAngle());
    }

}
