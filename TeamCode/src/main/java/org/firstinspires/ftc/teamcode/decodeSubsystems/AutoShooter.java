package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

public class AutoShooter {
    private static final double robotWidth = 18, robotLength = 18;//inch
    private static final double goalHeight = 40;//inch
    private static final double g = 386.1;//inch/s^2
    private static final Vector2d goalPos = new Vector2d(-62, 62);//red, 58
    private static final double[][] points ={{23,56,2400},{44.4,45.5,2600},{62,42,2900},{83.5,38,3200}, {100,38,3400},{118,38,3680},{135,38,3900},{150,36,4160}};//distance, angle, rpm
    private static final double[][] times = {{43.8,0.36},{61,0.46},{76,0.5},{85,0.5},{97,0.63},{107.2,0.63},{118.9,0.63},{124.2,0.67},{139.7,0.7}};

    //robot corners
    private static final Vector2d[] edges = {
            new Vector2d(robotWidth / 2, robotLength / 2),
            new Vector2d(-robotWidth / 2, robotLength / 2),
            new Vector2d(robotWidth / 2, -robotLength / 2),
            new Vector2d(-robotWidth / 2, -robotLength / 2),
            new Vector2d(robotWidth / 2, 0),
            new Vector2d(-robotWidth / 2, 0),
            new Vector2d(0, -robotLength / 2),
            new Vector2d(0, -robotLength / 2)};

    //is in launch zone
    public static boolean canLaunch(Pose2d robotPose) {
        double x = robotPose.position.x;
        double y = robotPose.position.y;
        double heading = robotPose.heading.toDouble();
        for (Vector2d p : edges) {
            p = p.rotateBy(Math.toDegrees(heading));
            if (isInZone(x + p.getX(), y + p.getY())) {
                return true;
            }
        }
        return false;

    }

    public static double getLaunchAngle(Pose2d robotPose, TeamColor teamColor) {//returns in degrees, 180 is towards motifs
        switch (teamColor) {
            case RED:
                return Math.toDegrees(Math.atan2(goalPos.getY() - robotPose.position.y, goalPos.getX() - robotPose.position.x) - robotPose.heading.toDouble());
            case BLUE:
                return -Math.toDegrees(Math.atan2(goalPos.getY() + robotPose.position.y, goalPos.getX() - robotPose.position.x) + robotPose.heading.toDouble());
        }
        return 180;
    }


    public static double[] getLaunchVector(Pose2d robotPose, TeamColor teamColor) {
//        return Math.sqrt(g * (Math.sqrt(Math.pow(getGoalDistance(robotPose, teamColor), 2) + Math.pow(goalHeight, 2)) + goalHeight));
        double distance = getGoalDistance(robotPose, teamColor);
        double[] min = {0, 0, 0};
        double[] max = {200, 0, 0};
        for (double[] point : points) {
            if (point[0] < distance & point[0] > min[0]) {
                min = point;
            } else if (point[0] < max[0]) {
                max = point;
            }
        }
        if (min[0] == 0) {
            return new double[]{max[1], max[2]};
        } else if (max[0] == 0) {
            return new double[]{min[1], min[2]};
        }
        double lowerRatio = distance - min[0];
        double higherRatio = max[0] - distance;
        double ratio = 1 / (max[0] - min[0]);

        return new double[]{(lowerRatio * max[1] + higherRatio * min[1]) * ratio, (lowerRatio * max[2] + higherRatio * min[2]) * ratio};

    }
    public static double getTime(Pose2d robotPose) {
//        return Math.sqrt(g * (Math.sqrt(Math.pow(getGoalDistance(robotPose, teamColor), 2) + Math.pow(goalHeight, 2)) + goalHeight));
        double distance = getGoalDistance(robotPose, SavedValues.teamColor);
        double[] min = {0, 0};
        double[] max = {200, 0};
        for (double[] point : times) {
            if (point[0] < distance & point[0] > min[0]) {
                min = point;
            } else if (point[0] < max[0]) {
                max = point;
            }
        }
        if (min[0] == 0) {
            return max[1];
        } else if (max[0] == 0) {
            return min[1];
        }
        double lowerRatio = distance - min[0];
        double higherRatio = max[0] - distance;
        double ratio = 1 / (max[0] - min[0]);

        return (lowerRatio * max[1] + higherRatio * min[1]) * ratio;

    }

    public static double getGoalDistance(Pose2d robotPose, TeamColor teamColor) {
        switch (teamColor) {
            case RED:
                return Math.hypot(robotPose.position.x - goalPos.getX(), robotPose.position.y - goalPos.getY());
            case BLUE:
                return Math.hypot(robotPose.position.x - goalPos.getX(), robotPose.position.y + goalPos.getY());
        }
        return 0;
    }

    private static boolean isInZone(double x, double y) {
        return Math.abs(y) < -x || 48 + Math.abs(y) < x;
    }

    public enum TeamColor {RED, BLUE}


}
