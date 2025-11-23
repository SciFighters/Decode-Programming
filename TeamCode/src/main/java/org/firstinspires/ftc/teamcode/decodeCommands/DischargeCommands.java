package org.firstinspires.ftc.teamcode.decodeCommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
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
        static final double kp = 0.028;
        public static double launchAngle;
        public static boolean shooting = false;

        public AutomaticAiming(DischargeSubsystem dischargeSubsystem, LimelightSubsystem limelightSubsystem, MecanumDrive mecanumDrive, CarouselSubsystem carouselSubsystem, AutoShooter.TeamColor teamColor) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.limelightSubsystem = limelightSubsystem;
            this.carouselSubsystem = carouselSubsystem;
            this.mecanumDrive = mecanumDrive;
            this.teamColor = teamColor;
//            this.flyWheelPower = flyWheelPower;
//            this.turretPower = turretPower;
//            this.rampDegree = rampDegree;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
//            aimTurret();
            if (AutoShooter.canLaunch(mecanumDrive.localizer.getPose()) || shooting) {
                double[] launchVector = AutoShooter.getLaunchVector(mecanumDrive.localizer.getPose(), teamColor);
                dischargeSubsystem.setRampDegree(launchVector[0]);
                dischargeSubsystem.setFlyWheelRPM(launchVector[1]);
            } else {
                dischargeSubsystem.setFlyWheelRPM(0);
            }

        }

        private void aimTurret() {


            launchAngle =
                    (AutoShooter.getLaunchAngle(new Pose2d(mecanumDrive.localizer.getPose().position, mecanumDrive.localizer.getPose().heading.toDouble() - Math.PI), teamColor) + 360) % 360;

            double power = -(launchAngle - dischargeSubsystem.getTurretAngle()) * kp;
            power += Math.signum(power) * 0.05;
            power = Range.clip(power, -0.4, 0.4);
            dischargeSubsystem.setTurretPower(power);
//            com.seattlesolvers.solverslib.geometry.Vector2d pos =
//                    limelightSubsystem.getRobotPos(mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw() / 180 * Math.PI, dischargeSubsystem.getTurretAngle());
//            double launchAngle =
//                    AutoShooter.getLaunchAngle(new Pose2d(pos.getX(), pos.getY(), mecanumDrive.lazyImu.get().getRobotYawPitchRollAngles().getYaw() / 180 * Math.PI), AutoShooter.TeamColor.RED);
//            if (pos.getX() < 1000) {
//                double power = -((launchAngle + 360) % 360 - dischargeSubsystem.getTurretAngle()) * 0.028;
//                power += Math.signum(power) * 0.04;
//                power = Range.clip(power,-0.4,0.4);
//                dischargeSubsystem.setTurretPower(power);
//            } else {
//                dischargeSubsystem.setTurretPower(0);
//            }

        }
    }
}
