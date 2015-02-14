
package ca.team3161.fenrir;

import ca.team3161.lib.robot.Drivetrain;
import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.utils.controls.Gamepad;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;

public class Robot extends TitanBot {

    private final RobotDrivetrain drivetrain;
    private final Gamepad gamepad;
    private final ToteElevator toteElevator;
    private final BinElevator binElevator;

    public Robot() {
        this.gamepad = new LogitechDualAction(0);
        this.drivetrain = new RobotDrivetrain(
                gamepad,
                new Drivetrain(new Talon(0)), // front left
                new Drivetrain(new Talon(1)), // front right
                new Drivetrain(new Talon(2)), // back left
                new Drivetrain(new Talon(3)), // back right,
                new Encoder(0, 1),
                new Encoder(2, 3),
                new Encoder(4, 5),
                new Encoder(10, 11),
                new Gyro(0)
                );
        this.toteElevator = new ToteElevator(
                new Drivetrain(new Talon(4)), // left elevator
                new Drivetrain(new Talon(5)), // right elevator
                new Drivetrain(new Talon(6)), // left intake
                new Drivetrain(new Talon(7)), // right intake
                new Encoder(8, 9),
                new Encoder(14, 15)
                );
        this.binElevator = new BinElevator(
                new Drivetrain(new Talon(8)),
                new Encoder(12, 13),
                new Solenoid(0)
                );
    }

    @Override
    public void autonomousRoutine() throws Exception {
    }

    @Override
    public int getAutonomousPeriodLengthSeconds() {
        return 15;
    }

    @Override
    public void robotInit() {
        gamepad.bind(LogitechButton.A, toteElevator::advanceElevator);
        gamepad.bind(LogitechButton.B, toteElevator::retreatElevator);
        gamepad.bind(LogitechButton.X, binElevator::advance);
        gamepad.bind(LogitechButton.Y, binElevator::retreat);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.PRESS, toteElevator::startIntake);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.RELEASE, toteElevator::stopIntake);
        gamepad.bind(LogitechButton.LEFT_TRIGGER, binElevator::deployClaw);
        gamepad.bind(LogitechButton.LEFT_BUMPER, binElevator::retractClaw);
    }

    @Override
    public void disabledInit() {
        gamepad.disableBindings();
        drivetrain.cancel();
        toteElevator.cancel();
        binElevator.cancel();
    }

    @Override
    public void teleopInit() {
        gamepad.enableBindings();
        drivetrain.start();
        toteElevator.start();
        binElevator.start();
    }

    @Override
    public void teleopRoutine() {
    }

}
