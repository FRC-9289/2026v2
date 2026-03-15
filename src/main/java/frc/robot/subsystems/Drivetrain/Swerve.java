package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.utils.TurretMath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    public RobotConfig config;
    public static Swerve swerve;

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }
        return swerve;
    }
    private Pose2d intialPose=new Pose2d(new Translation2d(0.0,0.0), new Rotation2d(0.0));

    // private final NetworkTable limelight =
    // NetworkTableInstance.getDefault().getTable("photonvision/Limelight");


    public Swerve() {
        //PP setup
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
        // Gyro setup
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration()
                .withMountPose(new MountPoseConfigs()));
        gyro.setYaw(0);
        Timer.delay(1);

        // Swerve Modules
        mSwerveMods = new SwerveModule[]{
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Odometry
        poseEstimator = new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(
                new Translation2d(-0.347,0.347),
                new Translation2d(0.347,0.347),
                new Translation2d(-0.347,-0.347),
                new Translation2d(0.347,-0.347)
            ),
            getGyroYaw(),
            getModulePositions(),
            new Pose2d()
        );

        // resetModulesToAbsolute();
    }
        


    /** Drive robot using x/y translation and rotation.
     * @param translation X/Y speed in meters/sec
     * @param rotation Rotation in rad/sec
     * @param fieldRelative True if robot should drive field-relative
     * @param isOpenLoop True if open-loop control, false for velocity control
     */

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
            getModuleStates()
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Alliance mirroring
        fieldRelative=true;
        Rotation2d allianceOffset = (DriverStation.getAlliance().equals(Alliance.Red))
                ? new Rotation2d(Math.PI)
                : new Rotation2d();

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getGyroYaw()
                  )
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        // Scale wheel speeds to max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, Constants.Swerve.maxSpeed
        );

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(states[mod.moduleNumber], false);
        }
    }

    /** Reset heading to 0 while keeping current position */
    public void zeroHeading() {
        setPose(new Pose2d(getPose().getTranslation(), new Rotation2d(0)));
    }

    /** Get current gyro yaw as Rotation2d */
    public Rotation2d getGyroYaw() {
        // return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
        return new Rotation2d(0);
    }

    /** Reset each module to its absolute encoder */
    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    /** Get current module positions */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /** Get current module states */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /** Get robot pose from odometry */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Reset odometry to specific pose */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void addLimelightVisionPose() {
        // Check if Limelight sees a valid target
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        if (!hasTarget) return;

        // Get bot pose from Limelight (field-relative, meters)
        double[] botPose = LimelightHelpers.getBotPose("limelight");
        if (botPose == null || botPose.length < 6) return;

        // Create Pose2d from Limelight data
        Pose2d visionPose = new Pose2d(
            botPose[0], // X in meters
            botPose[1], // Y in meters
            Rotation2d.fromRadians(botPose[5]) // yaw in radians
        );

        // Adjust timestamp for latency
        double latencySec = LimelightHelpers.getLatency_Capture("limelight") / 1000.0;
        double timestamp = Timer.getFPGATimestamp() - latencySec;

        // Add vision measurement to pose estimator
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), getModulePositions());
        // addLimelightVisionPose();

        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Heading", getPose().getRotation().getDegrees());

        // double[] botPose = LimelightHelpers.getBotPose_wpiBlue("limelight");
    }

    public void setInitialPose(Pose2d pose) {
        this.intialPose=pose;
        this.setPose(pose);
    }

    public Pose2d getInitialPose() {
        return this.intialPose;
    }
}