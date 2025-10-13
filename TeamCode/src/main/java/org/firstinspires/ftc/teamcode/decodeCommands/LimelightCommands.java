package org.firstinspires.ftc.teamcode.decodeCommands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

import java.util.function.Supplier;

public class LimelightCommands {
    public static class KalmanFilter extends CommandBase {
        public static Vector2d position;
        Vector2d pinpointPos;
        double positionCovarianceX = 0, positionCovarianceY = 0;
        final double pinpointKDistance = 0.01;
        double limelightCovarianceX, limelightCovarianceY;
        final double Kv = 0.005, Kd = 0.002, Kp = 0.001, Kf = 0.1;//for limelight covariance
        ElapsedTime time;
        double lastTime = 0;
        LimelightSubsystem limelightSubsystem;
        MecanumDrive mecanumDrive;
        final Supplier<Double> turretAngle;

        public KalmanFilter(LimelightSubsystem limelightSubsystem, MecanumDrive mecanumDrive, Supplier<Double> turretAngle) {
            this.limelightSubsystem = limelightSubsystem;
            this.mecanumDrive = mecanumDrive;
            position = new Vector2d(mecanumDrive.localizer.getPose().position.x, mecanumDrive.localizer.getPose().position.y);
            this.turretAngle = turretAngle;
        }

        public KalmanFilter(LimelightSubsystem limelightSubsystem, MecanumDrive mecanumDrive, Supplier<Double> turretAngle, double positionCovarianceX, double positionCovarianceY) {
            this.limelightSubsystem = limelightSubsystem;
            this.mecanumDrive = mecanumDrive;
            position = new Vector2d(mecanumDrive.localizer.getPose().position.x, mecanumDrive.localizer.getPose().position.y);
            this.positionCovarianceX = positionCovarianceX;
            this.positionCovarianceY = positionCovarianceY;
            this.turretAngle = turretAngle;

        }

        @Override
        public void initialize() {
            time = new ElapsedTime();
            pinpointPos = new Vector2d(mecanumDrive.localizer.getPose().position.x,mecanumDrive.localizer.getPose().position.y);
        }

        @Override
        public void execute() {
            double currentTime = time.seconds();
            com.acmerobotics.roadrunner.Vector2d mecanumDrivePos = mecanumDrive.localizer.getPose().position;
            Vector2d pinpointDelta = new Vector2d(mecanumDrivePos.x - pinpointPos.getX(), mecanumDrivePos.y - pinpointPos.getY());
            pinpointPos = new Vector2d(mecanumDrivePos.x, mecanumDrivePos.y);
            Vector2d limelightPos = limelightSubsystem.getRobotPos(mecanumDrive.localizer.getPose().heading.toDouble(), turretAngle.get());
            double pinpointCovarianceX = pinpointDelta.getX() * pinpointKDistance + 0.0001;
            double pinpointCovarianceY = pinpointDelta.getY() * pinpointKDistance + 0.0001;
            double limelightVelocityCovariance = Math.hypot(pinpointDelta.getX(), pinpointDelta.getY()) * Kv / (currentTime - lastTime);
            double limelightDistanceCovariance = Math.hypot(limelightSubsystem.aprilTagPos.getX() - position.getX(), limelightSubsystem.aprilTagPos.getY() - position.getY()) * Kd;
            double limelightPixelCovarianceX = limelightSubsystem.getPixelError().getX() * Kp;
            double limelightPixelCovarianceY = limelightSubsystem.getPixelError().getY() * Kp;
            double limelightCovarianceX = Kf + limelightVelocityCovariance + limelightDistanceCovariance + limelightPixelCovarianceX;
            double limelightCovarianceY = Kf + limelightVelocityCovariance + limelightDistanceCovariance + limelightPixelCovarianceY;
            position = new Vector2d(position.getX() + pinpointDelta.getX(), position.getY() + pinpointDelta.getY());//predict
            positionCovarianceX += pinpointCovarianceX;
            positionCovarianceY += pinpointCovarianceY;
            double kalmanGainX = positionCovarianceX / (positionCovarianceX + limelightCovarianceX);
            double kalmanGainY = positionCovarianceY / (positionCovarianceY + limelightCovarianceY);
            double positionX = position.getX() + kalmanGainX * (limelightPos.getX() - position.getX());
            double positionY = position.getY() + kalmanGainY * (limelightPos.getY() - position.getY());
            position = new Vector2d(positionX, positionY);
            lastTime = currentTime;
        }
    }
}
