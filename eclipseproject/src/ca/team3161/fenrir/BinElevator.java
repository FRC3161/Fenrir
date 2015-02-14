package ca.team3161.fenrir;

import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.RepeatingSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class BinElevator extends RepeatingSubsystem {

    private final SpeedController controller;
    private final Solenoid solenoid;
    private final Encoder encoder;

    public BinElevator(final SpeedController controller, final Encoder encoder, final Solenoid solenoid) {
        super(50, TimeUnit.MILLISECONDS);
        this.controller = controller;
        this.solenoid = solenoid;
        this.encoder = encoder;
    }

    @Override
    public void defineResources() {
        require(controller);
        require(solenoid);
        require(encoder);
    }

    private void set(final double rate) {
        controller.set(rate);
    }

    private void stop() {
        set(0);
    }

    private void waitAndStop() {
        try {
            Thread.sleep(500);
        } catch (final InterruptedException ie) {
        }
        stop();
    }

    public void advanceCommand() {
        set(0.5);
    }

    public void retreatCommand() {
        set(-0.5);
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
        // TODO: implements PIDs, use this to loop them
    }

}
