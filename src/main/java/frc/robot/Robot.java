package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.TurretMath;
import frc.robot.utils.Constants.TurretConstants;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final TurretSubsystem turret = new TurretSubsystem(true);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {

    // Record all NetworkTables entries
    DataLogManager.logNetworkTables(true);

    // Record Driver Station data (mode, alliance, etc.)
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();

    // ---- TURRET CONTROL LOOP ----
    // 1. Get current robot pose from drivetrain
    Pose2d pose = m_robotContainer.getDrivetrain().getPose();

    // 2. Compute desired turret angle (radians)
    double desiredAngle = TurretMath.getAngleToHub(pose);

    // 3. Compute error between target and current turret angle
    double error = desiredAngle - turret.getAngle();

    // 4. Compute desired velocity (P-controller)
    double kP = 4.0; // tune this for how fast you want the turret to move
    double desiredVelocity = kP * error;

    // 5. Compute desired acceleration (change in velocity per loop)
    double loopPeriod = 0.02; // 20ms, typical FRC loop
    double desiredAcceleration = (desiredVelocity - turret.getVelocity()) / loopPeriod;

    // 6. Update turret (sim or real hardware)
    turret.setVoltage(desiredVelocity, desiredAcceleration);
}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  private double turretPhysics(double w, double a) {
    return
      (TurretConstants.J_TURRET
      * TurretConstants.R)
      / (TurretConstants.GEAR_RATIO
      * TurretConstants.KT)
      * a
      + (TurretConstants.KE
      * TurretConstants.GEAR_RATIO)
      * w
      + TurretConstants.KS;
  }
}
