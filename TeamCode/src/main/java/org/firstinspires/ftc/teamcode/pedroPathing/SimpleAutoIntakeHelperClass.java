package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


// ToDo: Make sure to use the Smart Target Group for limelight in contour grouping to ensure the balls in a cluster
public class SimpleAutoIntakeHelperClass {
    private final Follower follower;
    private final Limelight3A limelight;

    // Proportional Driving Constants
    private final double STEER_K = 0.025;
    private final double DRIVE_K = 0.04;
    private final double TARGET_TY_STOP = -12.0; // Target vertical offset inside intake

    // Limelight constants for distance calculation
    private final double CAM_HEIGHT = 15.0;        // Height from ground to camera lens (inches)
    private final double TARGET_HEIGHT = 1.5;      // Height/radius of the game element (inches)
    private final double CAM_PITCH_DEGREES = 25.0; // Mounting angle of the limelight camera

    // Adjust to make the values tx and ty values of the cluster change more smooth or quicker.
    // Higher the values quicker they change lower the value smoother they change
    private final double FILTER_GAIN = 0.15;




    private double ballX = 0.0;
    private double ballY = 0.0;
    private double ballTA = 0.0;
    private boolean hasTarget = false;

    private PathChain toCluster;

    public enum SubState {
        TRAVELING_TO_CLUSTER,
        PROPORTIONAL_CHASE,
        DONE
    }
    private SubState currentState = SubState.DONE;

    public SimpleAutoIntakeHelperClass(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(100);
        this.limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            double rawX = result.getTx();
            double rawY = result.getTy();
            ballTA = result.getTa();
            hasTarget = (ballTA > 15.0);
            if (hasTarget){ // added to prevent jittering when doing the proportional driving by keeping the values smooth
                ballX = (rawX * FILTER_GAIN) + (ballX * (1.0 - FILTER_GAIN));
                ballY = (rawY * FILTER_GAIN) + (ballY * (1.0 - FILTER_GAIN));
            }
        } else {
            hasTarget = false;
        }

        // State Machine
        switch (currentState) {
            case TRAVELING_TO_CLUSTER:
                // Shift to vision control if Pedro finishes or limelight locks onto a close cluster
                if (!follower.isBusy() || (hasTarget && ballY > -5.0)) {
                    follower.breakFollowing();
                    currentState = SubState.PROPORTIONAL_CHASE;
                }
                break;

            case PROPORTIONAL_CHASE:
                if (!hasTarget) {
                    // Stop drivetrain if target disappears
                    follower.setTeleOpDrive(0, 0, 0, true);
                    currentState = SubState.DONE;
                    break;
                }

                // Tracking alignment
                double steerPower = ballX * STEER_K;
                double drivePower = (ballY - TARGET_TY_STOP) * DRIVE_K;

                // Protect motors from spikes
                drivePower = Math.min(Math.max(drivePower, -0.5), 0.5);

                // Send values to pedro pathing
                follower.setTeleOpDrive(drivePower, 0, steerPower, true);

                // Ball is now inside intake
                if (ballY <= TARGET_TY_STOP) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    currentState = SubState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    public void GlobalPositionFinder() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double BALLTX = result.getTx();
            double BALLTY = result.getTy();

            Pose RobotPose = follower.getPose();
            double RobotX = RobotPose.getX();
            double RobotY = RobotPose.getY();
            double RobotHeading = RobotPose.getHeading();

            // Calculate angle and distance
            double AngleToGoal = Math.toRadians(CAM_PITCH_DEGREES + BALLTY);
            double GoalDistance = (CAM_HEIGHT - TARGET_HEIGHT) / Math.tan(AngleToGoal);

            // Coordinates to go to
            double globalTargetAngle = RobotHeading + Math.toRadians(BALLTX);
            double targetX = RobotX + (GoalDistance * Math.cos(globalTargetAngle));
            double targetY = RobotY + (GoalDistance * Math.sin(globalTargetAngle));

            // Generate path
            toCluster = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(
                            new Pose(RobotX, RobotY),
                            new Pose(targetX, targetY)
                    )))
                    .setLinearHeadingInterpolation(RobotHeading, Math.toRadians(90))
                    .build();

            follower.followPath(toCluster);
            currentState = SubState.TRAVELING_TO_CLUSTER;
        }
    }

    // Used to see what this class is doing in our main opmode
    public SubState getCurrentState() {
        return currentState;
    }
}