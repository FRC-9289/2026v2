package frc.robot.ML;

public class NN {

    public static final double W = 1.19;
    public static final double B = 165.91;

    public double predict(double input){

        double output = W*input+B;

        return output;
    }
}