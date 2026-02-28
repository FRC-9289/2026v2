package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class Logitech extends Joystick {
    public interface Button {};
    public interface Axis {};

    enum POV implements Button {
        N,
        NE,
        NW,
        S,
        SE,
        SW,
        E,
        W,
    }

    enum RPAD implements Button {
        N,
        S,
        E,
        W
    }

    enum FRONTB implements Button {
        L,
        R,
    }

    enum FRONTD implements Axis {
        L,
        R
    }

    enum STICKB implements Button {
        L,
        R
    }

    enum STICKD implements Axis {
        L,
        R
    }

    enum ARROWS implements Button {
        LEFT,
        RIGHT
    }

    public Logitech() {
        super(0);
    }

    public boolean PoV(POV val) {
        return switch (val) {
            case N -> getPOV() == 0;
            case NE -> getPOV() == 45;
            case NW -> getPOV() == 315;
            case E -> getPOV() == 90;
            case S -> getPOV() == 180;
            case SE -> getPOV() == 135;
            case SW -> getPOV() == 225;
            case W -> getPOV() == 270;
        };
    }

    public boolean RPad(RPAD val) {
        return switch (val) {
            case N -> getRawButton(4);
            case E -> getRawButton(2);
            case S -> getRawButton(0);
            case W -> getRawButton(3);
        };
    }

    public boolean Front(FRONTB val) {
        return switch (val) {
            case L -> getRawButton(5);
            case R -> getRawButton(6);
        };
    }

    public double Front(FRONTD val) {
        return switch (val) {
            case L -> getRawAxis(2);
            case R -> getRawAxis(3);
        };
    }

    public boolean Stick(STICKB val) {
        return switch (val) {
            case L -> getRawButton(9);
            case R -> getRawButton(10);
        };
    }

    public double[] Stick(STICKD val) {
        double[] x = new double[2];
        switch (val) {
            case L -> {
                x[0] = getRawAxis(0);
                x[1] = -getRawAxis(1);
            }
            case R -> {
                x[0] = getRawAxis(4);
                x[1] = -getRawAxis(5);
            }
        }
        return x;
    }

    public boolean Arrows(ARROWS val) {
        return switch (val) {
            case LEFT -> getRawButton(7);
            case RIGHT -> getRawButton(8);
        };
    }
}
//Wolfram121