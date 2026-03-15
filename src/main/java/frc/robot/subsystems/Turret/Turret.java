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
  public static boolean operate=false;

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

    config.softLimit.forwardSoftLimit(1.8);
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimit(-0.8);
    config.softLimit.reverseSoftLimitEnabled(true);

    resetHeading();

    // config.closedLoop.pid(
    // TurretConstants.kP,
    // TurretConstants.kI,
    // TurretConstants.kD
    // );

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // force the limelight to pipeline 8 as soon as the robot turns on
    LimelightHelpers.setPipelineIndex("limelight", 8);
  }

  public double getHeadingRotations() 
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
    if (Math.abs(error) > 0.5)
    {
      motor.set(0.5);
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
    double angleToHub = TurretMath.getAngleToHubAdih(robotPose, hubLocation);
    return angleToHub;
  }

  @Override
  public void periodic() 
  {
    Pose2d robotPose = Swerve.getInstance().getPose();
    double angleToHub = autoRotateToHub();
    double motorRot = Units.degreesToRotations(angleToHub) * -TurretConstants.GEAR_RATIO;
    motorRot=(int)(motorRot*1000);
    motorRot/=1000;
    if(operate){
      double kS = 0.06*Math.signum(motorRot - motor.getEncoder().getPosition());
      double kV = 0.1*(motorRot - motor.getEncoder().getPosition());


      if(Math.abs(motorRot - motor.getEncoder().getPosition()) > 0.02
      
      
      
      ){
        SmartDashboard.putBoolean("Adjusting turret", true);
        setPower(kS+kV);
      } else {
        SmartDashboard.putBoolean("Adjusting turret", false);
        setPower(0);
      }
  }
    SmartDashboard.putNumber("Pose X", robotPose.getX());
    SmartDashboard.putNumber("Pose Y", robotPose.getY());
    SmartDashboard.putNumber("Heading", robotPose.getRotation().getDegrees());

    SmartDashboard.putNumber("AngleToHub", angleToHub);
    SmartDashboard.putNumber("Turret TargetMotorRot", motorRot);
    SmartDashboard.putNumber("Turret EncoderRot", motor.getEncoder().getPosition());
  }
}
