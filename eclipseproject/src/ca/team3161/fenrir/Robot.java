
package ca.team3161.fenrir;

import java.util.HashSet;
import java.util.Set;

import ca.team3161.fenrir.EncoderMonitor.LabelledEncoder;
import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.robot.pid.VelocityController;
import ca.team3161.lib.utils.controls.Gamepad;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TitanBot {

	public static final int MAX_DRIVETRAIN_RATE = 1500;
	public static final int MAX_ELEVATOR_RATE = 600;
	private final RobotDrivetrain drivetrain;
    private final Gamepad gamepad;
    private final ToteElevator toteElevator;
    private final BinElevator binElevator;
//    private final EncoderMonitor encoderMonitor;
    private final Preferences prefs = Preferences.getInstance();
    private final DigitalInput photoswitch = new DigitalInput(19);

    public Robot() {
        this.gamepad = new LogitechDualAction(0);

        final Encoder FLDriveEncoder = new Encoder(0, 1);
        final VelocityController FLDriveController = new VelocityController(new Drivetrain(new Talon(0)).setInverted(true),
        		FLDriveEncoder, MAX_DRIVETRAIN_RATE, 0, /*kI*/MAX_DRIVETRAIN_RATE/10, /*kD*/0, Float.MAX_VALUE, 1);
        
        final Encoder FRDriveEncoder = new Encoder(3, 2);
        final VelocityController FRDriveController = new VelocityController(new Drivetrain(new Talon(1)),
        		FRDriveEncoder, MAX_DRIVETRAIN_RATE, 0, /*kI*/MAX_DRIVETRAIN_RATE/10, /*kD*/0, Float.MAX_VALUE, 1);
        
        final Encoder BLDriveEncoder = new Encoder(4, 5);
        final VelocityController BLDriveController = new VelocityController(new Drivetrain(new Talon(2)).setInverted(true),
        		BLDriveEncoder, MAX_DRIVETRAIN_RATE, 0, /*kI*/MAX_DRIVETRAIN_RATE/10, /*kD*/0, Float.MAX_VALUE, 1);
        
        final Encoder BRDriveEncoder = new Encoder(11, 10);
        final VelocityController BRDriveController = new VelocityController(new Drivetrain(new Talon(3)),
        		BRDriveEncoder, MAX_DRIVETRAIN_RATE, 0, /*kI*/MAX_DRIVETRAIN_RATE/10, /*kD*/0, Float.MAX_VALUE, 1);

        final Gyro driveGyro = new Gyro(0);
        this.drivetrain = new RobotDrivetrain(
                gamepad,
                FLDriveController,
                FRDriveController,
                BLDriveController,
                BRDriveController,
                FLDriveEncoder,
                FRDriveEncoder,
                BLDriveEncoder,
                BRDriveEncoder,
                driveGyro
                );

        final Encoder leftElevatorEncoder = new Encoder(8, 9);
        final VelocityController leftElevatorController = new VelocityController(new Drivetrain(new Talon(4)).setInverted(false),
        		leftElevatorEncoder, MAX_ELEVATOR_RATE, 0.0015f, /*kI*/0.003f, /*kD*/0, 1200.0f, 1);
        
        final Encoder rightElevatorEncoder = new Encoder (15, 14);
        final VelocityController rightElevatorController = new VelocityController(new Drivetrain(new Talon(5)).setInverted(true),
        		rightElevatorEncoder, MAX_ELEVATOR_RATE, 0.0015f, /*kI*/0.003f, /*kD*/0, 1200.0f, 1);
        
        this.toteElevator = new ToteElevator(
                leftElevatorController,
                rightElevatorController,
                new Drivetrain(new Talon(6)).setInverted(true), 	//left intake
                new Drivetrain(new Talon(7)).setInverted(true), 	//right intake
                leftElevatorEncoder,
                rightElevatorEncoder,
                new Solenoid(1)
                );
        this.binElevator = new BinElevator(
                new Drivetrain(new Talon(8)).setInverted(true),
                new Encoder(12, 13),
                new Solenoid(0)
                );

//        final Set<LabelledEncoder> labelledEncoders = new HashSet<>();
//        labelledEncoders.add(new LabelledEncoder("FLDE", FLDriveEncoder));
//        labelledEncoders.add(new LabelledEncoder("FRDE", FRDriveEncoder));
//        labelledEncoders.add(new LabelledEncoder("BLDE", BLDriveEncoder));
//        labelledEncoders.add(new LabelledEncoder("BRDE", BRDriveEncoder));
//        this.encoderMonitor = new EncoderMonitor(labelledEncoders);
    }

    @Override
    public void autonomousRoutine() throws Exception {
        SmartDashboard.putString("Mode", "Auto Running");

        SmartDashboard.putString("Mode", "Auto Complete");
    }

    @Override
    public int getAutonomousPeriodLengthSeconds() {
        return 15;
    }

    @Override
    public void robotInit() {
        for (final LogitechControl control : LogitechControl.values()) {
            for (final LogitechAxis axis : LogitechAxis.values()) {
                gamepad.setMode(control, axis, d -> Math.abs(d) < 0.1 ? 0 : d); // deadband around [-0.1, 0.1]
            }
        }
        gamepad.bind(LogitechButton.A, PressType.PRESS, toteElevator::advanceElevatorCommand);
        gamepad.bind(LogitechButton.A, PressType.RELEASE, toteElevator::stopElevatorCommand);
        gamepad.bind(LogitechButton.B, PressType.PRESS, toteElevator::retreatElevatorCommand);
        gamepad.bind(LogitechButton.B, PressType.RELEASE, toteElevator::stopElevatorCommand);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.PRESS, toteElevator::startIntakeCommand);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.RELEASE, toteElevator::stopIntakeCommand);

        gamepad.bind(LogitechButton.X, binElevator::retreatCommand);
        gamepad.bind(LogitechButton.X, PressType.RELEASE, binElevator::stopCommand);
        gamepad.bind(LogitechButton.Y, binElevator::advanceCommand);
        gamepad.bind(LogitechButton.Y, PressType.RELEASE, binElevator::stopCommand);
        gamepad.bind(LogitechButton.LEFT_TRIGGER, binElevator::deployClawCommand);
        gamepad.bind(LogitechButton.LEFT_BUMPER, binElevator::retractClawCommand);

        SmartDashboard.putString("Mode", "On");

        //        CameraServer.getInstance().startAutomaticCapture();

//        encoderMonitor.start();
    }

    @Override
    public void disabledInit() {
        gamepad.disableBindings();
        drivetrain.cancel();
        toteElevator.cancel();
        binElevator.cancel();

        SmartDashboard.putString("Mode", "Disabled");
    }

    @Override
    public void teleopInit() {
        gamepad.enableBindings();
//        drivetrain.start();
        toteElevator.start();
        binElevator.start();

        SmartDashboard.putString("Mode", "Teleop Enabled");
    }

    @Override
    public void teleopRoutine() {
    	SmartDashboard.putNumber("LeftElev", toteElevator.getLeftEncoder().get());
    	SmartDashboard.putNumber("LeftElevRate", toteElevator.getLeftEncoder().getRate());
    	SmartDashboard.putNumber("RightElev", toteElevator.getRightEncoder().get());
    	SmartDashboard.putNumber("RightElevRate", toteElevator.getRightEncoder().getRate());
    	
    	SmartDashboard.putNumber("FL Wheel", drivetrain.getFLEncoder().get());
    	SmartDashboard.putNumber("FR Wheel", drivetrain.getFREncoder().get());
    	SmartDashboard.putNumber("BL Wheel", drivetrain.getBLEncoder().get());
    	SmartDashboard.putNumber("BR Wheel", drivetrain.getBREncoder().get());
    }

}
