package org.firstinspires.ftc.teamcode.needle.commands;

import org.firstinspires.ftc.teamcode.actions.ActionBase;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;

import java.util.function.Supplier;

public class ArmAxisCommands {
    public static class AxisGoTo extends ActionBase {
        ArmAxisSubsystem armAxisSubsystem;
        double angle;
        final double kp = 0.03;
        public AxisGoTo(ArmAxisSubsystem armAxisSubsystem, double angle){
            this.armAxisSubsystem = armAxisSubsystem;
            this.angle = angle;
            addRequirements(armAxisSubsystem);
        }

        @Override
        public void execute() {
            double power = (angle - armAxisSubsystem.getAngle()) * kp;
            armAxisSubsystem.setAxisPower(power);
        }

        @Override
        public boolean isFinished() {
            return angle - armAxisSubsystem.getAngle() < 5;
        }
    }

    public static class ResetAngle extends ActionBase{
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

    public static class AxisManual extends ActionBase{
        ArmAxisSubsystem armAxisSubsystem;
        Supplier<Double> power;
        public AxisManual(ArmAxisSubsystem armAxisSubsystem, Supplier<Double> power){
            this.armAxisSubsystem = armAxisSubsystem;
            this.power = power;
            addRequirements(armAxisSubsystem);
        }

        @Override
        public void execute() {
            armAxisSubsystem.setAxisPower(power.get());
        }


    }
}
