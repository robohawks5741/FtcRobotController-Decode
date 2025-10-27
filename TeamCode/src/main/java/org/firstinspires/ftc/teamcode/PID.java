package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    ElapsedTime time = new ElapsedTime();
    private double lastError = 0;

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * time.seconds();
        double derivative = (error - lastError) / time.seconds();
        lastError = error;

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        return output;
    }
}
