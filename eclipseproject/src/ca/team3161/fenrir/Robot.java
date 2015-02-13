
package ca.team3161.fenrir;

import ca.team3161.lib.robot.TitanBot;
import ca.team3161.lib.utils.controls.Gamepad;
import ca.team3161.lib.utils.controls.Gamepad.PressType;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechButton;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Talon;

public class Robot extends TitanBot {
    
    private final RobotDrivetrain drivetrain;
    private final Gamepad gamepad;
    private final ToteElevator toteElevator;
//    private final BinElevator binElevator;
    
    public Robot() {
        this.drivetrain = new RobotDrivetrain(
                new Talon(0),
                new Talon(1),
                new Talon(2),
                new Talon(3),
                new Gyro(0)
                );
        this.gamepad = new LogitechDualAction(0);
        this.toteElevator = new ToteElevator(new Talon(4), new Talon(5), new Talon(6), new Talon(7));
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
    }

    @Override
    public void teleopInit() {
        gamepad.enableBindings();
        gamepad.bind(LogitechButton.A, toteElevator::advance);
        gamepad.bind(LogitechButton.B, toteElevator::retreat);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.PRESS, toteElevator::startIntake);
        gamepad.bind(LogitechButton.RIGHT_TRIGGER, PressType.RELEASE, toteElevator::stopIntake);
    }

    @Override
    public void teleopRoutine() {
        drivetrain.drive(gamepad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.X),
                gamepad.getValue(LogitechControl.LEFT_STICK, LogitechAxis.Y),
                gamepad.getValue(LogitechControl.RIGHT_STICK, LogitechAxis.X));
    }

}
