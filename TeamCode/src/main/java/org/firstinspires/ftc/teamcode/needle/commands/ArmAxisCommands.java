package org.firstinspires.ftc.teamcode.needle.commands;

import org.firstinspires.ftc.teamcode.actions.ActionBase;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;

import java.util.function.Supplier;

public class ArmAxisCommands {
    public static class AxisGoTo extends ActionBase {
        ArmAxisSubsystem telescopicArmSubsystem;
        double angle;
        final double kp = 0.03;
        public AxisGoTo(ArmAxisSubsystem telescopicArmSubsystem, double angle){
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.angle = angle;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void execute() {
            double power = (angle - telescopicArmSubsystem.getAngle()) * kp;
            telescopicArmSubsystem.setAxisPower(power);
        }

        @Override
        public boolean isFinished() {
            return angle - telescopicArmSubsystem.getAngle() < 5;
        }
    }

    public static class ResetAngle extends ActionBase{
        ArmAxisSubsystem telescopicArmSubsystem;
        final double kp = 0.05;
        public ResetAngle(ArmAxisSubsystem telescopicArmSubsystem){
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void initialize() {
            double power = -telescopicArmSubsystem.getAngle() * kp;
            telescopicArmSubsystem.setAxisPower(power);
        }

        @Override
        public boolean isFinished() {
            return telescopicArmSubsystem.getAngle() < 5;
        }
    }

    public static class AxisManual extends ActionBase{
        ArmAxisSubsystem telescopicArmSubsystem;
        Supplier<Double> power;
        public AxisManual(ArmAxisSubsystem telescopicArmSubsystem, Supplier<Double> power){
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.power = power;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void execute() {
            telescopicArmSubsystem.setAxisPower(power.get());
        }


    }
}
