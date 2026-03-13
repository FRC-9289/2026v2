package frc.robot.ML;

public class NN {

    public double predict(double input){

        double minimum = 292.0;
        double maximum = 341.085;

        double output = 0.8532*input+197.132;

        return output;
    }
}