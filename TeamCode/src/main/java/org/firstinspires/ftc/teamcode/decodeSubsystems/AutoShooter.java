package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

public class AutoShooter {
    private static final double robotWidth = 18, robotLength = 15.5;//inch
    private static final double goalHeight = 40;//inch
    private static final double g = 386.1;//inch/s^2
    private static final Vector2d goalPos = new Vector2d(-60, 60);//red

    //robot corners
    private static final Vector2d[] points = {new Vector2d(robotWidth / 2, robotLength / 2),
            new Vector2d(-robotWidth / 2, robotLength / 2),
            new Vector2d(robotWidth / 2, -robotLength / 2),
            new Vector2d(-robotWidth / 2, -robotLength / 2)};

    //is in launch zone
    public static boolean canLaunch(Pose2d robotPose) {
        double x = robotPose.position.x;
        double y = robotPose.position.y;
        double heading = robotPose.heading.toDouble();
        for (Vector2d p : points) {
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
                return Math.toDegrees(Math.atan2(goalPos.getY() + robotPose.position.y, goalPos.getX() - robotPose.position.x) - robotPose.heading.toDouble());
        }
        return 180;
    }

    public static double getLaunchRampAngle(Pose2d robotPose, TeamColor teamColor) {//degrees
        double distance = getGoalDistance(robotPose, teamColor);
        return Math.toDegrees(Math.atan((goalHeight + Math.sqrt(Math.pow(distance, 2) + Math.pow(goalHeight, 2))) / distance));
    }

    public static double getLaunchVelocity(Pose2d robotPose, TeamColor teamColor) {//might be too low, returns in inch/s
        return Math.sqrt(g * (Math.sqrt(Math.pow(getGoalDistance(robotPose, teamColor), 2) + Math.pow(goalHeight, 2)) + goalHeight));
    }

    private static double getGoalDistance(Pose2d robotPose, TeamColor teamColor) {
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
