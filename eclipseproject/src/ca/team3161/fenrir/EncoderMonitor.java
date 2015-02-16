/*
 * @author Spok
 * Feb 15, 2015
 * 
 */
package ca.team3161.fenrir;

import java.util.Collection;
import java.util.HashSet;
import java.util.concurrent.TimeUnit;

import ca.team3161.lib.robot.subsystem.RepeatingIndependentSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EncoderMonitor extends RepeatingIndependentSubsystem {
	
	private final Collection<LabelledEncoder> encoders = new HashSet<>();

	public EncoderMonitor(final Collection<LabelledEncoder> encoders) {
		super(200, TimeUnit.MILLISECONDS);
		this.encoders.addAll(encoders);
	}

	@Override
	public void defineResources() {
	}

	@Override
	public void task() throws Exception {
		encoders.forEach(e -> SmartDashboard.putNumber(e.getLabel(), e.getEncoder().getRate()));
	}
	
	public static class LabelledEncoder {
		private final String label;
		private final Encoder encoder;
		
		public LabelledEncoder(final String label, final Encoder encoder) {
			this.label = label;
			this.encoder = encoder;
		}
		
		public String getLabel() {
			return label;
		}
		
		public Encoder getEncoder() {
			return encoder;
		}
	}

}
