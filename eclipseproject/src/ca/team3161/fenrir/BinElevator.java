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

    public void advance() {
    }

    public void retreat() {
    }

    public void deployClaw() {
    }

    public void retractClaw() {
    }

    @Override
    public void task() {
    }

}
