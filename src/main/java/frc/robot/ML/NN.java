package frc.robot.ML;

public class NN {

    static double relu(double x){
        return Math.max(0, x);
    }

    public double predict(double input){

        double[] w1 = {
            1.0393,0.6734,0.2655,-0.2203,
            0.7058,-0.9512,1.0312,0.2602
        };

        double[] b1 = {
            -0.0352,-0.2632,-0.2716,-0.4344,
            -0.2796,-0.6306,-0.4817,-0.9232
        };

        double[] w2 = {
            0.3691,0.5491,-0.3302,-0.3422,
            0.1799,-0.1524,0.2791,0.1227
        };

        double b2 = 0.0575;

        double minimum = 292.0;
        double maximum = 341.085;

        double standardized = (input - minimum) / (maximum - minimum);

        double[] hidden = new double[8];

        for(int i = 0; i < 8; i++){
            hidden[i] = relu(standardized * w1[i] + b1[i]);
        }

        double output = 0;

        for(int i = 0; i < 8; i++){
            output += hidden[i] * w2[i];
        }

        output += b2;

        double destandardized = output * (maximum - minimum) + minimum;

        return destandardized;
    }
}