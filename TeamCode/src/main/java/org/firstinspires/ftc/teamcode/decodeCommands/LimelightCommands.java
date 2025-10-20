package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

import java.util.function.Supplier;

public class LimelightCommands {
    @Config
    public static class KalmanFilter extends CommandBase {
        public static Vector2d position;
        Vector2d pinpointPos;
        double positionCovarianceX = 0, positionCovarianceY = 0;
        final double pinpointKDistance = 0.002;
        public static double Kv = 0.02, Kd = 0.18, Kp = 0.03, Kf = 0.7, Kz = 5;//for limelight covariance
        public static double kalmanGainX, kalmanGainY;
        ElapsedTime time;
        double lastTime = 0;
        LimelightSubsystem limelightSubsystem;
        MecanumDrive mecanumDrive;
        final Supplier<Double> turretAngle;
        final double llHeight = 0;//todo: change

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
            limelightSubsystem.setPipeline(1);
            limelightSubsystem.startLimelight();
            time = new ElapsedTime();
            pinpointPos = new Vector2d(mecanumDrive.localizer.getPose().position.x, mecanumDrive.localizer.getPose().position.y);
        }

        @Override
        public void execute() {
            double currentTime = time.seconds();
            com.acmerobotics.roadrunner.Vector2d mecanumDrivePos = mecanumDrive.localizer.getPose().position;
            Vector2d pinpointDelta = new Vector2d(mecanumDrivePos.x - pinpointPos.getX(), mecanumDrivePos.y - pinpointPos.getY());
            pinpointPos = new Vector2d(mecanumDrivePos.x, mecanumDrivePos.y);
            com.seattlesolvers.solverslib.geometry.Pose2d limelightPos3d = limelightSubsystem.getRobotPos(turretAngle.get());

            Vector2d pixelError = limelightSubsystem.getPixelError();
            if(limelightPos3d == null || pixelError == null){
                double pinpointCovarianceX = Math.abs(pinpointDelta.getX() * pinpointKDistance);
                double pinpointCovarianceY = Math.abs(pinpointDelta.getY() * pinpointKDistance);
                positionCovarianceX += pinpointCovarianceX;
                positionCovarianceY += pinpointCovarianceY;
                position = new Vector2d(position.getX() + pinpointDelta.getX(), position.getY() + pinpointDelta.getY());
                return;
            }
            Vector2d limelightPos = new Vector2d(limelightPos3d.getX(),limelightPos3d.getY());
            double pinpointCovarianceX = Math.abs(pinpointDelta.getX() * pinpointKDistance);
            double pinpointCovarianceY = Math.abs(pinpointDelta.getY() * pinpointKDistance);
            double limelightVelocityCovariance = 1 + Math.hypot(pinpointDelta.getX(), pinpointDelta.getY()) * Kv / (currentTime - lastTime);
            double limelightDistanceCovariance = 1 + Math.hypot(limelightSubsystem.aprilTagPos.getX() - position.getX(), limelightSubsystem.aprilTagPos.getY() - position.getY()) * Kd;
            double limelightPixelCovarianceX = 1 + Math.abs(pixelError.getX() * Kp);
            double limelightPixelCovarianceY = 1 + Math.abs(pixelError.getY() * Kp);
            double limelightCovarianceZ = 1 + Math.abs(limelightPos3d.getHeading() - llHeight) * Kz;
            double limelightCovarianceX = Kf * limelightVelocityCovariance * limelightDistanceCovariance * limelightPixelCovarianceX * limelightCovarianceZ;
            double limelightCovarianceY = Kf * limelightVelocityCovariance * limelightDistanceCovariance * limelightPixelCovarianceY * limelightCovarianceZ;
            position = new Vector2d(position.getX() + pinpointDelta.getX(), position.getY() + pinpointDelta.getY());//predict
            positionCovarianceX += pinpointCovarianceX;
            positionCovarianceY += pinpointCovarianceY;
            kalmanGainX = positionCovarianceX / (positionCovarianceX + limelightCovarianceX);
            kalmanGainY = positionCovarianceY / (positionCovarianceY + limelightCovarianceY);
            double positionX = position.getX() + kalmanGainX * (limelightPos.getX() - position.getX());
            double positionY = position.getY() + kalmanGainY * (limelightPos.getY() - position.getY());
            position = new Vector2d(positionX, positionY);
            positionCovarianceX *= (1 - kalmanGainX);
            positionCovarianceY *= (1 - kalmanGainY);
            lastTime = currentTime;
        }

        @Override
        public void end(boolean interrupted) {
            mecanumDrive.localizer.setPose(new Pose2d(position.getX(),position.getY(),mecanumDrive.localizer.getPose().heading.toDouble()));
            limelightSubsystem.stopLimelight();
        }
    }
}
