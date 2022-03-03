package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticHub;



/**
 * Add your docs here.
 */
public class PneumaticSubsystem extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public static PneumaticSubsystem instance = null;

  public final Solenoid leftIntake;
  public PneumaticHub pneumaticHub;

  public static PneumaticSubsystem getInstance() {
    if (instance == null) {
      instance = new PneumaticSubsystem();
    }
    return instance;
  }

  public PneumaticSubsystem() {
	  pneumaticHub = new PneumaticHub(1);
    leftIntake = new Solenoid(PneumaticsModuleType.REVPH, 8);        
	}

  public void extendIntake() {
    leftIntake.set(false);
  }

  public void retractIntake() {
    leftIntake.set(true);
  }

}