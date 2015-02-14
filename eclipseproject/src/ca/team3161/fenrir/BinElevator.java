package ca.team3161.fenrir;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.RepeatingSubsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;

public class BinElevator extends RepeatingSubsystem {

    private final SpeedController controller;
    private final Solenoid solenoid;
    private final Queue<Runnable> commandQueue = new ConcurrentLinkedQueue<>();

    public BinElevator(final SpeedController controller, final Solenoid solenoid) {
        super(50, TimeUnit.MILLISECONDS);
        this.controller = controller;
        this.solenoid = solenoid;
    }

    @Override
    public void defineResources() {
        require(controller);
        require(solenoid);
    }

    private void set(final double rate) {
        controller.set(rate);
    }

    public void advance() {
        commandQueue.offer(() -> {
            set(0.5);
            try {
                Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
            } catch (final InterruptedException ie) {
                // don't care
            }
            set(0);
        });
    }

    public void retreat() {
        commandQueue.offer(() -> {
            set(-0.5);
            try {
                Thread.sleep(TimeUnit.MILLISECONDS.toMillis(500));
            } catch (final InterruptedException ie) {
                // don't care
            }
            set(0);
        });
    }

    public void deployClaw() {
        commandQueue.offer(() -> {
            solenoid.set(true);
        });
    }

    public void retractClaw() {
        commandQueue.offer(() -> {
            solenoid.set(false);
        });
    }

    @Override
    public void task() {
        commandQueue.forEach(Runnable::run);
    }

}
