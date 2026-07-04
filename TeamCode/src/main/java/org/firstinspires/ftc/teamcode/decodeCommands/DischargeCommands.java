package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;


@Config
public class DischargeCommands {
    public static class setState extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        double flyWheelRPM;
        double rampDegree;
        double turretAngle;
        public static boolean canShoot;
        double kp = -0.02;

        public setState(DischargeSubsystem dischargeSubsystem, double flyWheelRPM, double rampDegree, double turretAngle) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelRPM = flyWheelRPM;
            this.rampDegree = rampDegree;
            this.turretAngle = turretAngle;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            canShoot = false;
            dischargeSubsystem.ramp.setRampDegree(rampDegree);
        }

        @Override
        public void execute() {
            dischargeSubsystem.flywheel.runRPM(flyWheelRPM);
            dischargeSubsystem.turret.runToAngleDirect(turretAngle);
            canShoot = Math.abs(dischargeSubsystem.flywheel.getRPM() - flyWheelRPM) < 300;
        }

    }

    @Config
    public static class AutomaticAiming extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        LimelightSubsystem limelightSubsystem;
        CarouselSubsystem carousel;
        MecanumDrive mecanumDrive;
        AutoShooter.TeamColor teamColor;
        public static double kp = 0.022;//0.018
        public static double ki = -0.00025;
        public static double kMovement = 2, effectSpeed = 19;
        public static double launchAngle;
        public static boolean shooting = false;
        public static boolean atSpeed = false;//+- 200rpm
        public static boolean atCloseSpeed = false;//+- 70rpm
        public static SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.06, 0.14, 0.08);
        public static double kd = -0.45;
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

        public AutomaticAiming(DischargeSubsystem dischargeSubsystem, LimelightSubsystem limelightSubsystem, MecanumDrive mecanumDrive, CarouselSubsystem carousel, AutoShooter.TeamColor teamColor) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.carousel = carousel;
            this.mecanumDrive = mecanumDrive;
            this.teamColor = teamColor;
            addRequirements(dischargeSubsystem);

        }

        @Override
        public void initialize() {
            turretCorrection = 0;
            rpmCorrection = 0;
            aim = true;
            limelight = true;
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
                double time = AutoShooter.getTime(mecanumDrive.localizer.getPose()) * kMovement;
                movementEffect = new com.seattlesolvers.solverslib.geometry.Vector2d(movement.linearVel.x * time, movement.linearVel.y * time)
                        .rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                if (movementEffect.magnitude() > effectSpeed) {
                    movementEffect = movementEffect.times(effectSpeed / movementEffect.magnitude());
                }
                acceleration = movementEffect.div(time).minus((lastMovement != null) ? lastMovement : movementEffect).times(time).div(deltaTime).times(ff.ka);
                aimTurret();

                if (AutoShooter.canLaunch(new Pose2d(new Vector2d(currentPos.position.x + mecanumToTurret.getX(),
                        currentPos.position.y + mecanumToTurret.getY()), currentPos.heading.toDouble()))) {
                    double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                            currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY()), currentPos.heading.toDouble()), teamColor);
                    dischargeSubsystem.ramp.setRampDegree(launchVector[0]);
                    dischargeSubsystem.flywheel.runRPM(launchVector[1] + rpmCorrection);
                    double delta = Math.abs(dischargeSubsystem.flywheel.getRPM() - (launchVector[1] + rpmCorrection));
                    atSpeed = delta < 120;
                    atCloseSpeed = delta < 70;
                } else if (shooting) {
                    com.seattlesolvers.solverslib.geometry.Vector2d closest = AutoShooter.closestPoint(
                            currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX() / time,
                            currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY() / time);
                    double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(closest.getX(),
                            closest.getY()), currentPos.heading.toDouble()), teamColor);
                    dischargeSubsystem.ramp.setRampDegree(launchVector[0]);
                    dischargeSubsystem.flywheel.runRPM(launchVector[1] + rpmCorrection);
                    double delta = Math.abs(dischargeSubsystem.flywheel.getRPM() - (launchVector[1] + rpmCorrection));
                    atSpeed = delta < 120;
                    atCloseSpeed = delta < 70;
                } else {
                    com.seattlesolvers.solverslib.geometry.Vector2d closest = AutoShooter.closestPoint(
                            currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                            currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY());
                    double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(closest.getX(),
                            closest.getY()), currentPos.heading.toDouble()), teamColor);
                    dischargeSubsystem.ramp.setRampDegree(launchVector[0]);
                    dischargeSubsystem.flywheel.keepRPM(launchVector[1] + rpmCorrection);
                    atSpeed = false;
                }
                lastPos = currentPos;
                lastMovement = movementEffect.div(time);
            } else {
                inRange = true;
                dischargeSubsystem.ramp.setRampDegree(41);
                dischargeSubsystem.flywheel.runRPM(3000 + rpmCorrection);
                dischargeSubsystem.turret.setPower(0);
                atSpeed = true;
                atCloseSpeed = true;
            }
            lastTime = currentTime;
        }

        private void aimTurret() {
            double angleError = limelightSubsystem.currentTx;
            double turretAngle = dischargeSubsystem.turret.getAngle();
            com.seattlesolvers.solverslib.geometry.Vector2d closest = AutoShooter.closestPoint(
                    currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                    currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY());
            double power;
            if (angleError != 0 && inRange && !(Math.abs(currentPos.position.y) > 40) && limelight) {
                double turretAngularVelocity = dischargeSubsystem.turret.getAngularVelocity();
                double aprilTagAngle = AutoShooter.getAprilTagAngle(new Pose2d(currentPos.position.x + mecanumToTurret.getX(),
                        currentPos.position.y + mecanumToTurret.getY(),
                        currentPos.heading.toDouble() - Math.PI), teamColor);
                double wantedError =
                        AutoShooter.getLaunchAngle(new Pose2d(currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX() + acceleration.getX(),
                                currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY() + acceleration.getY(),
                                currentPos.heading.toDouble() - Math.PI), teamColor) - aprilTagAngle;
                integral += (wantedError - angleError) * (currentTime - lastTime);
                power = -(wantedError - angleError) * kp;
                if (Math.signum(power) == Math.signum(integral)) {
                    integral = 0;
                }
                power += turretAngularVelocity * kd;
                power += integral * ki;

                power = (ff.ks > Math.abs(power)) ? Math.signum(power) * ff.ks : power;

            } else if (lastAngleError != 0 && angleError == 0) {
                integral = 0;

                llTime = time.seconds();
                power = 0;
            } else if (time.seconds() - llTime > 0.1) {
                double turretAngularVelocity = dischargeSubsystem.turret.getAngularVelocity();
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
                power += turretAngularVelocity * kd;
                power += Math.signum(power) * ff.ks;
            } else {
                power = 0;
                integral = 0;

            }
            lastAngleError = angleError;
            lastTurretAngle = turretAngle;
            power += movement.angVel * ff.kv;

            dischargeSubsystem.turret.setPower(power);


        }

        @Override
        public void end(boolean interrupted) {
            dischargeSubsystem.flywheel.setPower(0);
            dischargeSubsystem.turret.stopPower();
        }
    }
}