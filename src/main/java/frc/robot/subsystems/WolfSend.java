package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain.Swerve;

public class WolfSend extends SubsystemBase {
    private final NetworkTable t;
    private final NetworkTableEntry LFD;
    private final NetworkTableEntry LBD;
    private final NetworkTableEntry RBD;
    private final NetworkTableEntry RFD;
    private final NetworkTableEntry LFR1;
    private final NetworkTableEntry LBR1;
    private final NetworkTableEntry RBR1;
    private final NetworkTableEntry RFR1;
    private final NetworkTableEntry POS_X;
    private final NetworkTableEntry POS_Y;
    private final NetworkTableEntry ANGLE;
    private final NetworkTableEntry statusText;

    private final Swerve swerve = RobotContainer.getSwerve();
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    SwerveModuleState[] states = new SwerveModuleState[4];
    double[] desiredAngles = new double[4];
    private static final WolfSend wolfSend = new WolfSend();

    public WolfSend() {
        t = NetworkTableInstance.getDefault().getTable("BotTelemetry");

        LFD = t.getEntry("LFD");
        LBD = t.getEntry("LBD");
        RBD = t.getEntry("RBD");
        RFD = t.getEntry("RFD");
        LFR1 = t.getEntry("LFR1");
        LBR1 = t.getEntry("LBR1");
        RBR1 = t.getEntry("RBR1");
        RFR1 = t.getEntry("RFR1");
        POS_X = t.getEntry("POS_X");
        POS_Y = t.getEntry("POS_Y");
        ANGLE = t.getEntry("ANGLE");
        statusText = t.getEntry("status");
    }

    @Override
    public void periodic() {
        states = swerve.getModuleStates();
        positions = swerve.getModulePositions();

        LFD.setDouble(states[0].speedMetersPerSecond);
        LBD.setDouble(states[1].speedMetersPerSecond);
        RBD.setDouble(states[2].speedMetersPerSecond);
        RFD.setDouble(states[3].speedMetersPerSecond);
        LFR1.setDouble(positions[0].angle.getDegrees());
        LBR1.setDouble(positions[1].angle.getDegrees());
        RBR1.setDouble(positions[2].angle.getDegrees());
        RFR1.setDouble(positions[3].angle.getDegrees());
        POS_X.setDouble(swerve.getPose().getX());
        POS_Y.setDouble(swerve.getPose().getY());
        ANGLE.setDouble(swerve.getPose().getRotation().getDegrees());
        statusText.setString("Transmission Active");
    }

    public static WolfSend get() {
        return wolfSend;
    }
}
// Recieve with WolfRecieve.java