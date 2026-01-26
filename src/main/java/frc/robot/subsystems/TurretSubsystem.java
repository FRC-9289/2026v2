package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.controls.TurretSim;

public class TurretSubsystem extends SubsystemBase {

    private final PWMSparkMax motor = new PWMSparkMax(0);
    private final Encoder encoder = new Encoder(0, 1);
  
    private final TurretSim sim = new TurretSim();
  
    private final boolean isSim; // in simulation mode? - use Aditya's formula
  
    public TurretSubsystem(boolean isSim) {
      this.isSim = isSim;
  
      encoder.setDistancePerPulse(
          (2 * Math.PI) / (TurretConstants.GEAR_RATIO * 2048.0)
      );
    }
  
    public double getAngle() {
      return isSim ? sim.getAngle() : encoder.getDistance();
    }
  
    public double getVelocity() {
      return isSim ? sim.getVelocity() : encoder.getRate();
    }
  
    public void setVoltage(double volts, double angularAccel) {
      if (isSim) {
        sim.setVoltage(getVelocity(), angularAccel);
      } else {
        motor.setVoltage(volts);
      }
    }
  }