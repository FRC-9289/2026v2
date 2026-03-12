package frc.robot.subsystems.Turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.utils.TurretMath;
import frc.robot.utils.WolfSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

import java.nio.file.ClosedFileSystemException;
import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Turret extends SubsystemBase 
{

  private final WolfSparkMax motor;
  private final RelativeEncoder encoder;

  // public static Turret getInstance() {
  // return instance;
  // }

  public Turret() 
  {
    motor = new WolfSparkMax(
        TurretConstants.MOTOR_ID,
        true,
        false);

    encoder = motor.getEncoder();
    SparkMaxConfig config = new SparkMaxConfig();

    config.softLimit.forwardSoftLimit(-0.276759 * -TurretConstants.GEAR_RATIO);
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimit(-2);
    config.softLimit.reverseSoftLimitEnabled(true);

    // config.closedLoop.pid(
    // TurretConstants.kP,
    // TurretConstants.kI,
    // TurretConstants.kD
    // );

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetHeading();

    // force the limelight to pipeline 8 as soon as the robot turns on
    LimelightHelpers.setPipelineIndex("limelight", 8);
  }

  public double getHeadingOfMotorRad() 
  {
    return encoder.getPosition();
  }

  public void resetHeading() 
  {
    encoder.setPosition(0.0);
  }

  public void setDesiredAngle(double setpointRot) 
  {
    double measuredRot = motor.getEncoder().getPosition();
    double error = setpointRot - measuredRot;
    if (Math.abs(error) > Units.degreesToRotations(1) * TurretConstants.GEAR_RATIO) 
    {
      double error_rpm = error * 50 * 60;
      double regularized_rpm = MathUtil.clamp(error_rpm, -5676, 5676);
      motor.set(regularized_rpm / 5676);
    } else 
    {
      motor.set(0);
    }
  }

  public void runTest(double speed) 
  {
    motor.set(speed);
  }

  public void setPower(double speed) 
  {
    motor.set(speed);
  }

  public double autoRotateToHub(){

    Pose2d robotPose = Swerve.getInstance().getPose();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Translation2d hubLocation = new Translation2d(0.0,0.0);




    if(alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
      hubLocation = new Translation2d(TurretConstants.RED_HUB_X, TurretConstants.RED_HUB_Y);
    } else if(alliance.isPresent() && alliance.get().equals(Alliance.Blue)) {
      hubLocation = new Translation2d(TurretConstants.BLUE_HUB_X, TurretConstants.BLUE_HUB_Y);
    }
    double angleToHub = TurretMath.getAngleToHubAdih(robotPose, hubLocation);
    return angleToHub;
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Turret-Position",
        Units.rotationsToDegrees(getHeadingOfMotorRad() / -TurretConstants.GEAR_RATIO));
    Pose2d robotPose = Swerve.getInstance().getPose();
    double angleToHub = autoRotateToHub();
    //this.setDesiredAngle(Units.degreesToRotations(angleToHub * (-TurretConstants.GEAR_RATIO)));
    SmartDashboard.putNumber("Pose X", robotPose.getX());
    SmartDashboard.putNumber("Pose Y", robotPose.getY());
    SmartDashboard.putNumber("Heading", robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Turret-Calculated-Heading", angleToHub);
    // SmartDashboard.putNumber("ForwardLimit",
    // -0.276759*-TurretConstants.GEAR_RATIO);
    // SmartDashboard.putNumber("ReverseLimit",
    // 0.28797*-TurretConstants.GEAR_RATIO);
  }
}
