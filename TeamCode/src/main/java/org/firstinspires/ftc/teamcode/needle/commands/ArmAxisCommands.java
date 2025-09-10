package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

import java.util.function.Supplier;

public class ArmAxisCommands {
    @Config
    public static class AxisControl extends CommandBase {
        public enum Mode {GOTO, MANUAL}

        public static Mode mode = Mode.MANUAL;
        public static double targetPos = 0, lastAngle;// kp = 0.05, kd = 0.05;
        Supplier<Double> manual;
        ArmAxisSubsystem armAxisSubsystem;
        TelescopicArmSubsystem telescopicArmSubsystem;
        public static double kp = 1, kd = 0.08, kf = 0.9;
        public final double mass = 0.679;
        public final double g = 9.81;
        ElapsedTime time;
        double lastTime = 0;

        public AxisControl(ArmAxisSubsystem armAxisSubsystem, TelescopicArmSubsystem telescopicArmSubsystem, Supplier<Double> manual) {
            this.armAxisSubsystem = armAxisSubsystem;
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.manual = manual;
//            addRequirements(armAxisSubsystem);
        }

        @Override
        public void initialize() {
            lastAngle = armAxisSubsystem.getAngle();
            targetPos = 0;
            mode = Mode.MANUAL;
            time = new ElapsedTime();
            lastTime = time.seconds();
        }

        @Override
        public void execute() {
            double currentAngle = armAxisSubsystem.getAngle();
            double currentTime = time.seconds();

            switch (mode) {
                case GOTO:
                    if (Math.abs(manual.get()) > 0.15) {
                        armAxisSubsystem.setAxisPower(-manual.get());
                        armAxisSubsystem.breakMode();
                        mode = Mode.MANUAL;
                        break;
                    }
                    double power = (targetPos - armAxisSubsystem.getAngle()) * kp;
                    currentTime = time.seconds();
                    double derivative = (lastAngle - armAxisSubsystem.getAngle()) * kd * (currentTime - lastTime) * 100;
                    double feedForward =kf * mass * g * (telescopicArmSubsystem.getPosition() / (2 * telescopicArmSubsystem.getPosition() + 140)) * Math.cos(armAxisSubsystem.getAngle() * Math.PI / 180);
//                    double proportional = (targetPos - currentAngle) * kp;
//                    double derivative = (lastAngle - currentAngle) * kd;
//                    double feedforward = armAxisSubsystem.getCurrent() * Math.cos(Math.toRadians(armAxisSubsystem.getAngle())) / 5;
//                    double feedforward = (0.00064 * telescopicArmSubsystem.getPosition() + 0.0468) * Math.cos(armAxisSubsystem.getAngle() / 180 * Math.PI) * 1.2;
//                            Math.signum(Math.cos(armAxisSubsystem.getAngle() / 180 * Math.PI))
//                            * Math.sqrt(Math.abs(Math.cos(armAxisSubsystem.getAngle() / 180 * Math.PI)));

                    armAxisSubsystem.setAxisPower((power + derivative + feedForward)/30);

                    break;
                case MANUAL:
                    currentTime = time.seconds();
                    armAxisSubsystem.setAxisPower(-manual.get());
                    break;
            }
            lastAngle = currentAngle;
            lastTime = currentTime;
        }


        public static void setTargetPos(double targetPos1) {
            targetPos = targetPos1;
        }

        public static void setMode(Mode m) {
            mode = m;
        }
    }

    @Config
    public static class AxisGoTo extends CommandBase {
        ArmAxisSubsystem armAxisSubsystem;
        double wantedAngle;
        boolean wait = true;
        TelescopicArmSubsystem telescopicArmSubsystem;

        public AxisGoTo(ArmAxisSubsystem armAxisSubsystem, TelescopicArmSubsystem telescopicArmSubsystem, double wantedAngle) {
            this.armAxisSubsystem = armAxisSubsystem;
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.wantedAngle = wantedAngle;

        }

        public AxisGoTo(ArmAxisSubsystem armAxisSubsystem, TelescopicArmSubsystem telescopicArmSubsystem, double wantedAngle, boolean wait) {
            this.armAxisSubsystem = armAxisSubsystem;
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.wantedAngle = wantedAngle;

        }


        @Override
        public void initialize() {
            AxisControl.setTargetPos(wantedAngle);
            AxisControl.setMode(AxisControl.Mode.GOTO);
        }


        @Override
        public boolean isFinished() {
            return Math.abs(wantedAngle - armAxisSubsystem.getAngle()) < 3 || !wait || AxisControl.mode == AxisControl.Mode.MANUAL;
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
