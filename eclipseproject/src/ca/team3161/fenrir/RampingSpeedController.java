package ca.team3161.fenrir;

import static ca.team3161.lib.utils.Utils.normalizePwm;

import java.util.Objects;

import ca.team3161.lib.utils.Assert;
import edu.wpi.first.wpilibj.SpeedController;

public class RampingSpeedController implements SpeedController {

    private final SpeedController controller;
    private final double maxStep;

    public RampingSpeedController(final SpeedController controller, final double maxStep) {
        Assert.assertTrue("Maximum step size cannot be less than -1 or greater than 1", Math.abs(maxStep) <= 1);
        Objects.requireNonNull(controller);
        this.controller = controller;
        this.maxStep = maxStep;
    }

    @Override
    public void pidWrite(final double output) {
        controller.pidWrite(normalizePwm(adjust(normalizePwm(output))));
    }

    @Override
    public double get() {
        return normalizePwm(controller.get());
    }

    @Override
    public void set(final double speed, final byte syncGroup) {
        controller.set(normalizePwm(adjust(normalizePwm(speed))), syncGroup);
    }

    @Override
    public void set(final double speed) {
        controller.set(normalizePwm(adjust(normalizePwm(speed))));
    }

    @Override
    public void disable() {
        controller.disable();
    }

    private double adjust(final double target) {
        final double error = target - get();
        if (Math.abs(target) <= 0.05) {
            if (Math.abs(target) <= 0.005) {
                return 0;
            } else {
                return get()/1.02;
            }
        }
        if (error > maxStep) {
            return get() + maxStep;
        } else if (error < -maxStep) {
            return get() - maxStep;
        } else {
            return target;
        }
    }

}
