package org.firstinspires.ftc.teamcode.needle.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;

import java.util.function.Supplier;
public class ArmAxisCommands {

    @Config
    public static class AxisGoTo extends CommandBase {
        ArmAxisSubsystem armAxisSubsystem;
        double wantedAngle, lastAngle;
        public static double kp = 0.05, kd = 0.0;
        public AxisGoTo(ArmAxisSubsystem armAxisSubsystem, double wantedAngle){
            this.armAxisSubsystem = armAxisSubsystem;
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

            armAxisSubsystem.setAxisPower(power + derivative);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(wantedAngle - armAxisSubsystem.getAngle()) < 3;
        }
    }

    public static class ResetAngle extends CommandBase{
        ArmAxisSubsystem armAxisSubsystem;
        final double kp = 0.05;
        public ResetAngle(ArmAxisSubsystem armAxisSubsystem){
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

    public static class waitForAngle extends CommandBase{
        ArmAxisSubsystem armAxisSubsystem;
        double angle;
        boolean forward;
        public waitForAngle(ArmAxisSubsystem armAxisSubsystem, double angle){
            this.armAxisSubsystem = armAxisSubsystem;
            this.angle = angle;
        }

        @Override
        public void initialize() {
            forward = angle < armAxisSubsystem.getAngle();
        }

        @Override
        public boolean isFinished() {
            if (forward){
                return  angle > armAxisSubsystem.getAngle();
            }
            return  angle < armAxisSubsystem.getAngle();

        }
    }

    public static class AxisManual extends CommandBase{
        ArmAxisSubsystem armAxisSubsystem;
        Supplier<Double> power;
        public AxisManual(ArmAxisSubsystem armAxisSubsystem, Supplier<Double> power){
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
