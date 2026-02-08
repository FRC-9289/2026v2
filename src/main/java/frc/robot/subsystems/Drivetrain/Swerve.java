package frc.robot.subsystems.Drivetrain;

import frc.robot.SwerveModule;
import edu.wpi.first.math.estimator.PoseEstimator;
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
import edu.wpi.first.units.Units;
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

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics swerveDriveKinematics;

    public final Pose2d rt = new Pose2d( /* r(t) = desired position at time t */
        new Translation2d(0.0,1.0), // desired position
        Rotation2d.fromDegrees(0) // desired heading
    );


    public Swerve() {
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        }
        catch(IOException | ParseException e){
            e.printStackTrace();
            config = new RobotConfig(0, 0, null, 0);
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(
                        Constants.Swerve.PPtranslationkP,
                        Constants.Swerve.PPtranslationkI,
                        Constants.Swerve.PPtranslationkD
                        ), // Translation PID constants
                        new PIDConstants(
                        Constants.Swerve.PProtationalkP,
                        Constants.Swerve.PProtationalkI,
                        Constants.Swerve.PProtationalkD
                        ) // Rotation PID constants
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
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "Drivetrain");
        gyro.getConfigurator().apply(new Pigeon2Configuration()
                .withMountPose(new MountPoseConfigs().withMountPoseYaw(180)));
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
        swerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(-0.345, 0.345),//FL
            new Translation2d(0.345, 0.345),//FR
            new Translation2d(-0.345, -0.345),//BL
            new Translation2d(0.345, -0.345)//BR
        );
        poseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, getGyroYaw(), getModulePositions(), getPose());

        // resetModulesToAbsolute();
    }
        


    /** Drive robot using x/y translation and rotation.
     * @param translation X/Y speed in meters/sec
     * @param rotation Rotation in rad/sec
     * @param fieldRelative True if robot should drive field-relative
     * @param isOpenLoop True if open-loop control, false for velocity control
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Alliance mirroring
        Rotation2d allianceOffset = (DriverStation.getAlliance().equals(Alliance.Red))
                ? new Rotation2d(Math.PI)
                : new Rotation2d();

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getGyroYaw().rotateBy(allianceOffset)
                  )
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
        );

        // Scale wheel speeds to max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /** Reset heading to 0 while keeping current position */
    public void zeroHeading() {
        setPose(new Pose2d(getPose().getTranslation(), new Rotation2d(0)));
    }

    /** Get current gyro yaw as Rotation2d */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-gyro.getYaw().getValueAsDouble());
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

    @Override
    public void periodic() {
        poseEstimator.update(getGyroYaw(), getModulePositions());

        Pose2d pose = getPose();
        SmartDashboard.putNumber("Pose X", pose.getX());
        SmartDashboard.putNumber("Pose Y", pose.getY());
        SmartDashboard.putNumber("Heading", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("Desired X", rt.getX());
        SmartDashboard.putNumber("Desired Y", rt.getY());
        SmartDashboard.putNumber("Desired Heading", rt.getRotation().getDegrees());
    }

    // Robot-relative speeds supplier used by AutoBuilder
    private ChassisSpeeds getRobotRelativeSpeeds() {
        // Compute chassis speeds from current module states (robot-relative)
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    // Drive using robot-relative ChassisSpeeds (used by AutoBuilder)
    private void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            // Use closed-loop (false -> velocity control) so path following uses velocity control
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }
}

