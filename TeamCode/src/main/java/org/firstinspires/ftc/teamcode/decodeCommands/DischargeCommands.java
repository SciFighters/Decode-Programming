package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

import java.util.function.Supplier;

@Config
public class DischargeCommands {

    public static class setState extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        double flyWheelRPM;
        double rampDegree;
        double wantedPos;
        public static boolean canShoot;
        static final double kp = 0.01;

        public setState(DischargeSubsystem dischargeSubsystem, double flyWheelRPM, double rampDegree) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelRPM = flyWheelRPM;
            this.rampDegree = rampDegree;

            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            canShoot = false;
            dischargeSubsystem.setRampDegree(rampDegree);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setFlyWheelRPM(flyWheelRPM);
            double power = (wantedPos - dischargeSubsystem.getTurretPosition()) * kp;
            canShoot = Math.abs(dischargeSubsystem.getRPM() - flyWheelRPM) < 300;
        }

    }

    @Config
    public static class AutomaticAiming extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        LimelightSubsystem limelightSubsystem;
        CarouselSubsystem carouselSubsystem;
        MecanumDrive mecanumDrive;
        AutoShooter.TeamColor teamColor;
        public static double kp = 0.018;//0.018
        public static double ki = 0.0;
        public static double launchAngle;
        public static boolean shooting = false;
        public static boolean atSpeed = false;//+- 200rpm
        public static boolean atCloseSpeed = false;//+- 70rpm
        public static double kv = 0.14, ks = 0.06, kd = -0.36;
        private com.seattlesolvers.solverslib.geometry.Vector2d mecanumToTurret;
        public static boolean aim = true;
        public static boolean inRange = true;
        public static boolean limelight = true;
        public static double turretCorrection = 0, rpmCorrection = 0;
        private com.seattlesolvers.solverslib.geometry.Vector2d movementEffect;
        Pose2d currentPos, lastPos = new Pose2d(0, 0, 0);
        ElapsedTime time;
        double llTime = 0, lastAngleError, lastTurretAngle;
        PoseVelocity2d movement;
        com.seattlesolvers.solverslib.geometry.Vector2d lastMovement, acceleration;
        double currentTime, lastTime, deltaTime;
        double integral = 0;

        public AutomaticAiming(DischargeSubsystem dischargeSubsystem, LimelightSubsystem limelightSubsystem, MecanumDrive mecanumDrive, CarouselSubsystem carouselSubsystem, AutoShooter.TeamColor teamColor) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.carouselSubsystem = carouselSubsystem;
            this.mecanumDrive = mecanumDrive;
            this.teamColor = teamColor;
            addRequirements(dischargeSubsystem);

        }

        @Override
        public void initialize() {
            turretCorrection = 0;
            rpmCorrection = 0;
            time = new ElapsedTime();
            limelight = true;
            lastTime = time.seconds();
        }

        @Override
        public void execute() {
            if (aim) {
                currentTime = time.seconds();
                deltaTime = currentTime - lastTime;
                movement = mecanumDrive.localizer.update();

                mecanumToTurret = new com.seattlesolvers.solverslib.geometry.Vector2d(1.5748, 0).rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                currentPos = mecanumDrive.localizer.getPose();
                double time = AutoShooter.getTime(mecanumDrive.localizer.getPose()) * 2;
                movementEffect = new com.seattlesolvers.solverslib.geometry.Vector2d(movement.linearVel.x * time, movement.linearVel.y * time)
                        .rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                acceleration = movementEffect.div(time).minus((lastMovement != null) ? lastMovement : movementEffect).times(time * time / 2).div(deltaTime).times(0.002);
                aimTurret();

                if (AutoShooter.canLaunch(new Pose2d(new Vector2d(currentPos.position.x + mecanumToTurret.getX(),
                        currentPos.position.y + mecanumToTurret.getY()), currentPos.heading.toDouble()))) {
                    double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                            currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY()), currentPos.heading.toDouble()), teamColor);
                    dischargeSubsystem.setRampDegree(launchVector[0]);
                    dischargeSubsystem.setFlyWheelRPM(launchVector[1] + rpmCorrection);
                    double delta = Math.abs(dischargeSubsystem.getRPM() - (launchVector[1] + rpmCorrection));
                    atSpeed = delta < 200;
                    atCloseSpeed = delta < 70;
                } else if (shooting) {
                    com.seattlesolvers.solverslib.geometry.Vector2d closest = AutoShooter.closestPoint(
                            currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX() / time,
                            currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY() / time);
                    double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(closest.getX(),
                            closest.getY()), currentPos.heading.toDouble()), teamColor);
                    dischargeSubsystem.setRampDegree(launchVector[0]);
                    dischargeSubsystem.setFlyWheelRPM(launchVector[1] + rpmCorrection);
                    double delta = Math.abs(dischargeSubsystem.getRPM() - (launchVector[1] + rpmCorrection));
                    atSpeed = delta < 200;
                    atCloseSpeed = delta < 70;
                } else {
                    com.seattlesolvers.solverslib.geometry.Vector2d closest = AutoShooter.closestPoint(
                            currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                            currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY());
                    double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(closest.getX(),
                            closest.getY()), currentPos.heading.toDouble()), teamColor);
                    dischargeSubsystem.setRampDegree(launchVector[0]);
                    dischargeSubsystem.stayRPM(launchVector[1] + rpmCorrection);
                    atSpeed = false;
                }
                lastPos = currentPos;
                lastMovement = movementEffect.div(time);
            } else {
                inRange = true;
                dischargeSubsystem.setRampDegree(41);
                dischargeSubsystem.setFlyWheelRPM(3000);
                dischargeSubsystem.setTurretPower(0);
                atSpeed = true;
                atCloseSpeed = true;
            }
            lastTime = currentTime;
        }

        private void aimTurret() {
            double angleError = limelightSubsystem.currentTx;
            double turretAngle = dischargeSubsystem.getTurretAngle();
            com.seattlesolvers.solverslib.geometry.Vector2d closest = AutoShooter.closestPoint(
                    currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                    currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY());
            double power;
            if (angleError != 0 && inRange && !(Math.abs(currentPos.position.y) > 40) && limelight) {

                double rps = dischargeSubsystem.getRPS();
                double aprilTagAngle = AutoShooter.getAprilTagAngle(new Pose2d(currentPos.position.x + mecanumToTurret.getX(),
                        currentPos.position.y + mecanumToTurret.getY(),
                        currentPos.heading.toDouble() - Math.PI), teamColor);
                double wantedError =
                        AutoShooter.getLaunchAngle(new Pose2d(currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX() + acceleration.getX(),
                                currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY() + acceleration.getY(),
                                currentPos.heading.toDouble() - Math.PI), teamColor) - aprilTagAngle;
                integral += angleError * (currentTime - lastTime);
                power = -(wantedError - angleError) * kp;
                power += rps * kd;
                power += integral * ki;

                power = (ks > Math.abs(power)) ? Math.signum(power) * ks : power;

            } else if (lastAngleError != 0 && angleError == 0) {
                integral = 0;

                llTime = time.seconds();
                power = 0;
            } else if (time.seconds() - llTime > 0.1) {
                launchAngle = (AutoShooter.canLaunch(currentPos)) ?
                        (AutoShooter.getLaunchAngle(new Pose2d(currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                                currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY(),
                                currentPos.heading.toDouble() - Math.PI), teamColor) + 360 + turretCorrection) % 360 :
                        (AutoShooter.getLaunchAngle(new Pose2d(closest.getX() + mecanumToTurret.getX(),
                                closest.getY() + mecanumToTurret.getY(),
                                currentPos.heading.toDouble() - Math.PI), teamColor) + 360 + turretCorrection) % 360;
                inRange = launchAngle > 20 && launchAngle < 340;
                launchAngle = Range.clip(launchAngle, 20, 340);
                integral = 0;
                power = -(launchAngle - turretAngle) * kp;
                power += Math.signum(power) * ks;
            } else {
                power = 0;
                integral = 0;

            }
            lastAngleError = angleError;
            lastTurretAngle = turretAngle;
            power += movement.angVel * kv;

            dischargeSubsystem.setTurretPower(power);


        }

        @Override
        public void end(boolean interrupted) {
            dischargeSubsystem.setFlyWheelPower(0);
            dischargeSubsystem.setTurretPower(0);
        }
    }
}
