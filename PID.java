package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double p = 0;
    double i = 0;
    double d = 0;
    double target = 0;
    double integralStorage = 0;
    double previousTime = 0;
    ElapsedTime time = new ElapsedTime();
    double previousError = 0;

    PID(double proportion, double integral, double derivative) {
        p = proportion;
        i = integral;
        d = derivative;
        time.reset();


    }

    public void setTarget(double target) {
        this.target = target;
    }

    public double pidUpdate(double currentPos) {
        double currentTime = time.seconds();
        double error = target - currentPos;
        double proportional = p * error;
        double integralStorage = +i * error;
        double errorChange = Math.abs( error - previousError);
        double timeChange = Math.abs(currentTime - previousTime);
        double derivative = d * (errorChange/timeChange);
        previousError = error;
        previousTime = currentTime;
        time.reset();
        return proportional+integralStorage+derivative;
    }

}
