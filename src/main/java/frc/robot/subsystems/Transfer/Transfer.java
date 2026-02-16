package frc.robot.subsystems.Transfer;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Transfer extends SubsystemBase {

    private SparkMax transfer1;
    private SparkMax transfer2;
    private double speed = 1.0;

    public Transfer() {

        transfer1 = new SparkMax(Constants.TransferConstants.TRANSFER_MOTOR_ID_1,MotorType.kBrushless);

        transfer2 = new SparkMax(Constants.TransferConstants.TRANSFER_MOTOR_ID_2,MotorType.kBrushless);
    }

    public void movingForward() {
            transfer1.set(speed); 
            transfer2.set(-speed);
    }
    public void movingBackward() {
            transfer1.set(-speed); 
            transfer2.set(speed);
    }
    public void stop() {
        transfer1.stopMotor();
        transfer2.stopMotor();
    }

    private static Transfer instance;
    public static Transfer getInstance(){
        return instance;
    }
}