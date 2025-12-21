package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;

import java.util.function.Supplier;

@Config
public class DischargeCommands {
    public static class DischargeManual extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        Supplier<Double> flyWheelPower;
        Supplier<Double> turretPower;
        Supplier<Double> rampDegree;

        public DischargeManual(DischargeSubsystem dischargeSubsystem, Supplier<Double> flyWheelPower, Supplier<Double> turretPower, Supplier<Double> rampDegree) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.flyWheelPower = flyWheelPower;
            this.turretPower = turretPower;
            this.rampDegree = rampDegree;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            dischargeSubsystem.setFlyWheelPower(flyWheelPower.get());
            dischargeSubsystem.setTurretPower(turretPower.get());
            dischargeSubsystem.setRampDegree(rampDegree.get());
        }
    }

    public static class setState extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        double flyWheelRPM;
        double rampDegree;
        double wantedPos;
        public static boolean canShoot;//variable for knowing if you can shout, will be removed later with automatic shooting
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
        //        Supplier<Double> flyWheelPower;
//        Supplier<Double> turretPower;
//        Supplier<Double> rampDegree;
        double wantedPos;
        static final double kp = 0.02;
        public static double launchAngle;
        public static boolean shooting = false;
        public static boolean atSpeed = false;
        public static double kv = 0.12, ks = 0.055;
        private com.seattlesolvers.solverslib.geometry.Vector2d mecanumToTurret;
        private Pose2d currentPos;
        public static boolean aim = true;
        public static double turretCorrection = 0, rpmCorrection = 0;

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
        }

        @Override
        public void execute() {
            if(aim){
                mecanumToTurret = new com.seattlesolvers.solverslib.geometry.Vector2d(1.5748, 0).rotateBy(mecanumDrive.localizer.getPose().heading.toDouble() / Math.PI * 180);
                currentPos = mecanumDrive.localizer.getPose();

                aimTurret();

                if (AutoShooter.canLaunch(new Pose2d(new Vector2d(mecanumDrive.localizer.getPose().position.x + mecanumToTurret.getX(),
                        mecanumDrive.localizer.getPose().position.y + mecanumToTurret.getY()), currentPos.heading.toDouble())) || shooting) {
                    double[] launchVector = AutoShooter.getLaunchVector(mecanumDrive.localizer.getPose(), teamColor);
                    dischargeSubsystem.setRampDegree(launchVector[0]);
                    dischargeSubsystem.setFlyWheelRPM(launchVector[1] + rpmCorrection);
                    atSpeed = Math.abs(dischargeSubsystem.getRPM() - launchVector[1]) < 40;
                } else {
                    dischargeSubsystem.setFlyWheelRPM(0);
                    atSpeed = false;
                }
            }
            else {
                dischargeSubsystem.setRampDegree(40);
                dischargeSubsystem.setFlyWheelRPM(2600);
                atSpeed = true;
            }

        }

        private void aimTurret() {
//            double pixelError = limelightSubsystem.getTx();
            double mecanumSpeed = mecanumDrive.localizer.update().angVel;

            launchAngle =
                    (AutoShooter.getLaunchAngle(new Pose2d(mecanumDrive.localizer.getPose().position.x + mecanumToTurret.getX(),
                            mecanumDrive.localizer.getPose().position.y + mecanumToTurret.getY(),
                            mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI), teamColor) + 360 + turretCorrection) % 360;

            double power;
//            if (limelightSubsystem.getTx() != 0 && dischargeSubsystem.getTurretAngle() < 355 && dischargeSubsystem.getTurretAngle() > 5){
//                power = limelightSubsystem.getTx() * kp;
//            } else{
            power = -(launchAngle - dischargeSubsystem.getTurretAngle()) * kp;
//            }
            power += mecanumSpeed * kv;
            power += Math.signum(power) * ks;
            dischargeSubsystem.setTurretPower(power);


        }
    }
}
