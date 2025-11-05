package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double integralSum = 0;

    ElapsedTime time = new ElapsedTime();
    private double lastError = 0;

    //reference = target, state = current value, Kp = magnitude of correction, Kd = damper, Ki = long tern correction magnitude
    public double PIDControl(double Kp, double Ki, double Kd, double reference, double state){
        double error = reference - state;
        integralSum += error * time.seconds();
        double derivative = (error - lastError) / time.seconds();
        lastError = error;

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        return output;
    }
}
