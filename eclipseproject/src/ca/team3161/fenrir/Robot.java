
package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.robot.pid.VelocityController;
import ca.team3161.lib.utils.controls.DeadbandJoystickMode;
import ca.team3161.lib.utils.controls.Gamepad;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TitanBot {

	public static final int MAX_DRIVETRAIN_RATE = 1500;
	public static final int MAX_ELEVATOR_RATE = 600;
	private RobotDrivetrain drivetrain;
    private final Gamepad gamepad;
    private ToteElevator toteElevator;
    private BinElevator binElevator;
    private final DigitalInput binElevatorTopSwitch = new DigitalInput(18);
    private final Preferences prefs = Preferences.getInstance();
    private int webcamSession = 0;
    private Image webcamImage = null;

    public Robot() {
        this.gamepad = new LogitechDualAction(0, 10, TimeUnit.MILLISECONDS);

        setupDrivetrain();
        setupToteElevator();
        setupBinElevator();
    }

	private void setupBinElevator() {
		this.binElevator = new BinElevator(
                new Drivetrain(new Talon(8)).setInverted(true),
                new Encoder(12, 13),
                new Solenoid(0),
                new DigitalInput(17),
                binElevatorTopSwitch
                );
	}

	private void setupToteElevator() {
		final float kP = 0.0015f;
		final float kI = 0.003f;
		final float kD = 0;
		final float maxI = 1200;
		final float deadband = 1;
		final Encoder leftElevatorEncoder = new Encoder(8, 9);
        final VelocityController leftElevatorController = new VelocityController(new Drivetrain(new Talon(4)).setInverted(false),
        		leftElevatorEncoder, MAX_ELEVATOR_RATE, kP, kI, kD, maxI, deadband);

        final Encoder rightElevatorEncoder = new Encoder (15, 14);
        final VelocityController rightElevatorController = new VelocityController(new Drivetrain(new Talon(5)).setInverted(true),
        		rightElevatorEncoder, MAX_ELEVATOR_RATE, kP, kI, kD, maxI, deadband);

        this.toteElevator = new ToteElevator(
                leftElevatorController,
                rightElevatorController,
                new Drivetrain(new Talon(6)).setInverted(true), 	//left intake
                new Drivetrain(new Talon(7)).setInverted(true), 	//right intake
                leftElevatorEncoder,
                rightElevatorEncoder,
                new DigitalInput(19),
                new DigitalInput(20),
                new Solenoid(1)
                );
	}

	private void setupDrivetrain() {
		final float kP = 0.00135f;
        final float kI = 0.0002f;
        final float kD = 0;
        final float maxI = 4000;
        final float deadband = 150;
        final double maxStep = 0.01;
        final Encoder FLDriveEncoder = new Encoder(0, 1);
        final SpeedController FLDriveController = getDriveController(new Talon(0), true, kP, kI, kD, FLDriveEncoder, MAX_DRIVETRAIN_RATE, maxI, deadband, maxStep);

        final Encoder FRDriveEncoder = new Encoder(3, 2);
        final SpeedController FRDriveController = getDriveController(new Talon(1), false, kP, kI, kD, FRDriveEncoder, MAX_DRIVETRAIN_RATE, maxI, deadband, maxStep);

        final Encoder BLDriveEncoder = new Encoder(4, 5);
        final SpeedController BLDriveController = getDriveController(new Talon(2), true, kP, kI, kD, BLDriveEncoder, MAX_DRIVETRAIN_RATE, maxI, deadband, maxStep);

        final Encoder BRDriveEncoder = new Encoder(11, 10);
        final SpeedController BRDriveController = getDriveController(new Talon(3), false, kP, kI, kD, BRDriveEncoder, MAX_DRIVETRAIN_RATE, maxI, deadband, maxStep);

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
	}

    public static SpeedController getDriveController(final SpeedController baseController, final boolean inverted,
    		final float kP, final float kI, final float kD,
    		final Encoder encoder,
    		final int maxRate, final float maxI, final float deadband, final double maxStep) {
    	return new RampingSpeedController(
	    			new VelocityController(
		    			new Drivetrain(baseController)
		    			.setInverted(inverted),
	        		encoder, maxRate, kP, kI, kD, maxI, deadband),
        		maxStep);
    }

    @Override
    public void autonomousRoutine() throws Exception {
    	if (!binElevatorTopSwitch.get()) {
    		return;
    	}
    	drivetrain.start();
    	toteElevator.start();
    	drivetrain.setAutonomous(true);
    	toteElevator.closeClaws();
    	toteElevator.advanceElevatorCommand();
    	waitFor(1, TimeUnit.SECONDS);
    	drivetrain.setAutoRates(0.7, -0.7);
    	toteElevator.setIntake(-ToteElevator.INTAKE_MOTOR_PWM);
    	waitFor(1, TimeUnit.SECONDS);
    	toteElevator.stopIntake();
    	waitFor(3500, TimeUnit.MILLISECONDS);
    	drivetrain.setAutoRates(0, 0);
    	drivetrain.setAutonomous(false);
    }

    @Override
    public int getAutonomousPeriodLengthSeconds() {
        return 15;
    }

    @Override
    public void robotInit() {
    	webcamImage = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
    	
    	webcamSession = NIVision.IMAQdxOpenCamera("cam0",
    			NIVision.IMAQdxCameraControlMode.CameraControlModeController);
    	NIVision.IMAQdxConfigureGrab(webcamSession);
    	
        for (final LogitechControl control : LogitechControl.values()) {
            for (final LogitechAxis axis : LogitechAxis.values()) {
                gamepad.setMode(control, axis, d -> Math.abs(d) < 0.05 ? 0 : d); // deadband around [-0.05, 0.05]
            }
        }
        gamepad.bind(LogitechButton.A, PressType.PRESS, this::intake);
        gamepad.bind(LogitechButton.B, PressType.PRESS, toteElevator::retreatElevatorCommand);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.PRESS, toteElevator::startIntakeCommand);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.RELEASE, toteElevator::stopIntakeCommand);

        gamepad.bind(LogitechButton.X, binElevator::retreatCommand);
        gamepad.bind(LogitechButton.X, PressType.RELEASE, binElevator::stopCommand);
        gamepad.bind(LogitechButton.Y, binElevator::advanceCommand);
        gamepad.bind(LogitechButton.Y, PressType.RELEASE, binElevator::stopCommand);
        gamepad.bind(LogitechButton.LEFT_TRIGGER, binElevator::deployClawCommand);
        gamepad.bind(LogitechButton.LEFT_BUMPER, binElevator::retractClawCommand);
        
        gamepad.bind(LogitechButton.RIGHT_BUMPER, PressType.PRESS, drivetrain::enableTurbo);
        gamepad.bind(LogitechButton.RIGHT_BUMPER, PressType.RELEASE, drivetrain::disableTurbo);
    }
    
    private void intake() {
    	binElevator.retractClawCommand();
    	toteElevator.advanceElevatorCommand();
    }

    @Override
    public void disabledInit() {
    	NIVision.IMAQdxStopAcquisition(webcamSession);
        gamepad.disableBindings();
        drivetrain.cancel();
        toteElevator.cancel();
        binElevator.cancel();
    }

    @Override
    public void teleopInit() {
    	NIVision.IMAQdxStartAcquisition(webcamSession);
    	drivetrain.setAutonomous(false);
        gamepad.enableBindings();
        drivetrain.start();
        toteElevator.start();
        binElevator.start();
        toteElevator.openClaws();
    }

    @Override
    public void teleopRoutine() {
    	renderWebcam();
    	SmartDashboard.putNumber("FL Enc", drivetrain.getFLEncoder().getRate());
    	SmartDashboard.putNumber("FR Enc", drivetrain.getFREncoder().getRate());
    	SmartDashboard.putNumber("BL Enc", drivetrain.getBLEncoder().getRate());
    	SmartDashboard.putNumber("BR Enc", drivetrain.getBREncoder().getRate());
    }
    
    private void renderWebcam() {
    	try {
        	final NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);
        	NIVision.IMAQdxGrab(webcamSession, webcamImage, 1);
        	NIVision.imaqDrawShapeOnImage(webcamImage, webcamImage, rect,
        			DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
        	
        	CameraServer.getInstance().setImage(webcamImage);    		
    	} catch (final Exception e) {
    		e.printStackTrace();
    	}
    }

}
