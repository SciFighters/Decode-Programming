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
        static final double kp = 0.022;//0.018
        public static double launchAngle;
        public static boolean shooting = false;
        public static boolean atSpeed = false;//+- 200rpm
        public static boolean atCloseSpeed = false;//+- 70rpm
        public static double kv = 0.12, ks = 0.04;
        private com.seattlesolvers.solverslib.geometry.Vector2d mecanumToTurret;
        public static boolean aim = true;
        public static boolean inRange = true;
        public static double turretCorrection = 0, rpmCorrection = 0;
        private com.seattlesolvers.solverslib.geometry.Vector2d movementEffect;
        Pose2d currentPos;
        ElapsedTime time;
        double llTime = 0, lastAngleError;
        PoseVelocity2d movement;

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
        }

        @Override
        public void execute() {
            if(aim){
                movement = mecanumDrive.localizer.update();
                mecanumToTurret = new com.seattlesolvers.solverslib.geometry.Vector2d(1.5748, 0).rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                currentPos = mecanumDrive.localizer.getPose();
                double time = AutoShooter.getTime(mecanumDrive.localizer.getPose());
                movementEffect = new com.seattlesolvers.solverslib.geometry.Vector2d(movement.linearVel.x * time,movement.linearVel.y * time).rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                aimTurret();

                double[] launchVector = AutoShooter.getLaunchVector(new Pose2d(new Vector2d(mecanumDrive.localizer.getPose().position.x + mecanumToTurret.getX() + movementEffect.getX(),
                        mecanumDrive.localizer.getPose().position.y + mecanumToTurret.getY() + movementEffect.getY()), currentPos.heading.toDouble()), teamColor);
                dischargeSubsystem.setRampDegree(launchVector[0]);
                if (shooting || AutoShooter.canLaunch(new Pose2d(new Vector2d(mecanumDrive.localizer.getPose().position.x + mecanumToTurret.getX(),
                        mecanumDrive.localizer.getPose().position.y + mecanumToTurret.getY()), currentPos.heading.toDouble()))) {
                    dischargeSubsystem.setFlyWheelRPM(launchVector[1] + rpmCorrection);
                    double delta = Math.abs(dischargeSubsystem.getRPM() - (launchVector[1] + rpmCorrection));
                    atSpeed = delta < 200;
                    atCloseSpeed =  delta < 70;
                } else {
                    dischargeSubsystem.stayRPM(launchVector[1] + rpmCorrection);
                    atSpeed = false;
                }
            }
            else {
                inRange = true;
                dischargeSubsystem.setRampDegree(41);
                dischargeSubsystem.setFlyWheelRPM(3000);
                dischargeSubsystem.setTurretPower(0);
                atSpeed = true;
                atCloseSpeed = true;
            }

        }

        private void aimTurret() {
            double angleError = limelightSubsystem.getTx();
            double turretAngle = dischargeSubsystem.getTurretAngle();

            double power;
            if (angleError != 0 && inRange && !(Math.abs(currentPos.position.y) > 40)){
                double wantedError = AutoShooter.getWantedTx(new Pose2d(currentPos.position.x + mecanumToTurret.getX(),
                        currentPos.position.y + mecanumToTurret.getY(),
                        currentPos.heading.toDouble() - Math.PI), teamColor);
                power = -(wantedError - angleError) * 0.014;
                power += Math.signum(power) * 0.045;


            } else if(lastAngleError != 0 && angleError == 0){
                llTime = time.seconds();
                power = 0;
            }else  if(time.seconds() - llTime > 0.1){
                launchAngle =
                        (AutoShooter.getLaunchAngle(new Pose2d(currentPos.position.x + mecanumToTurret.getX() + movementEffect.getX(),
                                currentPos.position.y + mecanumToTurret.getY() + movementEffect.getY(),
                                currentPos.heading.toDouble() - Math.PI), teamColor) + 360 + turretCorrection) % 360;
                inRange = launchAngle > 50 && launchAngle < 290;
                launchAngle = Range.clip(launchAngle,50,290);
                power = -(launchAngle - turretAngle) * kp;
                power += Math.signum(power) * ks;
            }else{
                power = 0;
            }
            lastAngleError = angleError;
            power += movement.angVel * kv;

            dischargeSubsystem.setTurretPower(power);


        }
    }
}
