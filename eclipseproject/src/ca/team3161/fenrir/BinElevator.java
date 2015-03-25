package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingPooledSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class BinElevator extends RepeatingPooledSubsystem {

    private static final double MOTOR_PWM = 0.75;
    private volatile double pwmTarget = 0;
    private final SpeedController controller;
    private final Solenoid solenoid;
    private final Encoder encoder;
    private final DigitalInput bottomLimitSwitch, topLimitSwitch;

    public BinElevator(final SpeedController controller, final Encoder encoder, final Solenoid solenoid,
            final DigitalInput bottomLimitSwitch, final DigitalInput topLimitSwitch) {
        super(10, TimeUnit.MILLISECONDS);
        this.controller = controller;
        this.solenoid = solenoid;
        this.encoder = encoder;
        this.bottomLimitSwitch = bottomLimitSwitch;
        this.topLimitSwitch = topLimitSwitch;
    }

    @Override
    public void defineResources() {
        require(controller);
        require(solenoid);
        require(encoder);
    }

    private void set(final double rate) {
        pwmTarget = rate;
    }

    private void stop() {
        set(0);
    }

    public void advanceCommand() {
        set(MOTOR_PWM);
    }

    public void retreatCommand() {
        set(-MOTOR_PWM);
    }

    public void stopCommand() {
        stop();
    }

    public void deployClawCommand() {
        solenoid.set(true);
    }

    public void retractClawCommand() {
        solenoid.set(false);
    }

    @Override
    public void task() {
        final boolean bottomSwitch = !bottomLimitSwitch.get(), topSwitch = !topLimitSwitch.get();
        if (pwmTarget > 0 && topSwitch) {
            stop();
        } else if (pwmTarget < 0 && bottomSwitch) {
            stop();
        } else {
            controller.set(pwmTarget);
        }
    }

}
