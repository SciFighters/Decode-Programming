package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

import java.util.function.Supplier;

public class ArmAxisCommands {
//    public static class AxisControl extends CommandBase {
//        public enum Mode {GOTO, MANUAL}
//        public static Mode mode = Mode.MANUAL;
//        private static double targetPos = 0, kp = 0.05;
//        ArmAxisSubsystem armAxisSubsystem;
//
//        public AxisControl(ArmAxisSubsystem armAxisSubsystem, Supplier<Double> manual) {
//            this.armAxisSubsystem = armAxisSubsystem;
//        }
//
//        @Override
//        public void execute() {
//            switch (mode){
//                case GOTO:
//                    armAxisSubsystem.setAxisPower();
//                    break;
//                case MANUAL:
//
//                    break;
//            }
//        }
//
//
//        public static void setTargetPos(double targetPos1) {
//            targetPos = targetPos1;
//        }
//
//        public static void setMode(Mode m) {
//            mode = m;
//        }
//    }

    @Config
    public static class AxisGoTo extends CommandBase {
        ArmAxisSubsystem armAxisSubsystem;
        double wantedAngle, lastAngle;
        public static double kp = 11.0, kd = 4.0, kf = 1;
        public final double mass = 0.679;
        public final double g = 9.81;
        TelescopicArmSubsystem telescopicArmSubsystem;

        public AxisGoTo(ArmAxisSubsystem armAxisSubsystem, TelescopicArmSubsystem telescopicArmSubsystem, double wantedAngle) {
            this.armAxisSubsystem = armAxisSubsystem;
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.wantedAngle = wantedAngle;

            addRequirements(armAxisSubsystem);
        }


        @Override
        public void initialize() {
            lastAngle = armAxisSubsystem.getAngle();
        }

        @Override
        public void execute() {
            double power = (wantedAngle - armAxisSubsystem.getAngle()) * kp;
            double derivative = (lastAngle - armAxisSubsystem.getAngle()) * kd;
            double feedForward = kf * mass * g * (telescopicArmSubsystem.getPosition() / 200) * Math.cos(armAxisSubsystem.getAngle());

            armAxisSubsystem.setAxisPower((power + derivative + feedForward)/30);

            lastAngle = armAxisSubsystem.getAngle();
        }

        @Override
        public boolean isFinished() {
            return Math.abs(wantedAngle - armAxisSubsystem.getAngle()) < 1;
        }
    }

    public static class ResetAngle extends CommandBase {
        ArmAxisSubsystem armAxisSubsystem;
        final double kp = 0.05;

        public ResetAngle(ArmAxisSubsystem armAxisSubsystem) {
            this.armAxisSubsystem = armAxisSubsystem;
            addRequirements(armAxisSubsystem);
        }

        @Override
        public void initialize() {
            double power = -armAxisSubsystem.getAngle() * kp;
            armAxisSubsystem.setAxisPower(power);
        }

        @Override
        public boolean isFinished() {
            return armAxisSubsystem.getAngle() < 5;
        }
    }

    public static class waitForAngle extends CommandBase {
        ArmAxisSubsystem armAxisSubsystem;
        double angle;
        boolean forward;

        public waitForAngle(ArmAxisSubsystem armAxisSubsystem, double angle) {
            this.armAxisSubsystem = armAxisSubsystem;
            this.angle = angle;
        }

        @Override
        public void initialize() {
            forward = angle < armAxisSubsystem.getAngle();
        }

        @Override
        public boolean isFinished() {
            if (forward) {
                return angle > armAxisSubsystem.getAngle();
            }
            return angle < armAxisSubsystem.getAngle();

        }
    }

    public static class AxisManual extends CommandBase {
        ArmAxisSubsystem armAxisSubsystem;
        Supplier<Double> power;

        public AxisManual(ArmAxisSubsystem armAxisSubsystem, Supplier<Double> power) {
            this.armAxisSubsystem = armAxisSubsystem;
            this.power = power;
            addRequirements(armAxisSubsystem);
        }

        @Override
        public void execute() {
            armAxisSubsystem.setAxisPower(-power.get());
        }


    }
}
