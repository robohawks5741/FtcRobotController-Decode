package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double integralSum = 0;
    public double indexIntegralSum = 0;

    public ElapsedTime time = new ElapsedTime();
    public ElapsedTime indexTime = new ElapsedTime();
    public double indexLastError = 0;
    public double lastError = 0;

    //reference = target, state = current value, Kp = magnitude of correction, Kd = damper, Ki = long tern correction magnitude
    public double PIDControl(double Kp, double Ki, double Kd, double reference, double state){

        double error = reference - state;
        integralSum += error * time.seconds();
        double derivative = (error - lastError) / time.seconds();
        lastError = error;

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        return output;
    }
    public double indexPID(double kP, double kI, double kD, double target, double current) {
        // --- Wraparound Logic ---
        // Calculate the shortest path to the target angle.
        double error = target - current;
       /*if (Math.abs(error) > 180) {
            if (error > 0 ) {
                error -= 360;
            } else {
                error += 360;
            }
        } */
        // --- End Wraparound ---

        double dt = indexTime.seconds();
        // Prevent division by zero on the first loop
        if (Double.isNaN(dt) || dt == 0) {
            dt = 1e-6;
        }

        // PID calculations
        double derivative = (error - indexLastError) / dt;
        indexIntegralSum += error * dt;

        double output = (kP * error) + (kI * indexIntegralSum) + (kD * derivative);

        // Update state for the next loop
        indexLastError = error;
        indexTime.reset();

        return output;
    }
    public void reset() {
        integralSum = 0;
        lastError = 0;
        time.reset();
    }
    public void indexReset() {
        indexIntegralSum = 0;
        indexLastError = 0;
        indexTime = new ElapsedTime();
    }
}
