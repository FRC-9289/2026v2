package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Transfer extends SubsystemBase {

    private SparkMax transfer1;
    private SparkMax transfer2;
    private double speedB =-0.6;
    private double speedF =0.6;

    public Transfer() {

        transfer1 = new SparkMax(Constants.TransferConstants.TRANSFER_MOTOR_ID_1,MotorType.kBrushless);

        transfer2 = new SparkMax(Constants.TransferConstants.TRANSFER_MOTOR_ID_2,MotorType.kBrushless);
    }

    public void movingForward() {
            transfer1.set(speedF); 
            transfer2.set(-speedF);
    }
    public void movingBackward() {
            transfer1.set(speedB); 
            transfer2.set(-speedB);
    }
    public void stop() {
        transfer1.stopMotor();
        transfer2.stopMotor();
    }
}