package org.firstinspires.ftc.teamcode.pedroPathing;


import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;


public class SharedClass {


    public static String motif = "PPG";
    public static double xPos = 7.5;
    public static double yPos = 7.8;
    public static double yaw = 90;


    public static double pos1Shoot;
    public static double pos2Shoot;
    public static double pos3Shoot;
    public static double increment;


    public static double pos1Intake;
    public static double pos2Intake;
    public static double pos3Intake;

    public static double turretPose;


    public static void loadCalibration() {
        try {
            BufferedReader reader = new BufferedReader(
                    new FileReader(AppUtil.getInstance().getSettingsFile("calibrationInfo.txt"))
            );


            pos1Shoot = Double.parseDouble(reader.readLine());
            pos2Shoot = Double.parseDouble(reader.readLine());
            pos3Shoot = Double.parseDouble(reader.readLine());
            pos1Intake = Double.parseDouble(reader.readLine());
            pos2Intake = Double.parseDouble(reader.readLine());
            pos3Intake = Double.parseDouble(reader.readLine());


            if (pos1Shoot > 0.5) {
                increment = -0.2055;
            } else {
                increment = 0.2055;
            }






            reader.close();


        } catch (Exception e) {
            // DO NOT crash robot
            System.out.println("Calibration file failed to load");
        }
    }
}

