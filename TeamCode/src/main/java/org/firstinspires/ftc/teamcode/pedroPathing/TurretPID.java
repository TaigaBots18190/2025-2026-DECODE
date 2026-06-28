package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;


public class TurretPID {
    public double Kp = 0.0014;
    public double Ki = 0.0;
    public double Kd = 0.000021;
    public double Kf = 0.065;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();


    public double calculatePower(double setpoint, double currentPosition) {
        double error = setpoint - currentPosition;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * Math.signum(error));

        lastError = error;
        timer.reset();
        return out;
    }
}