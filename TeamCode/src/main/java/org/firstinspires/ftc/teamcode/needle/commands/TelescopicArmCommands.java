package org.firstinspires.ftc.teamcode.needle.commands;


import org.firstinspires.ftc.teamcode.actions.ActionBase;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

import java.util.function.Supplier;

public class TelescopicArmCommands {
    public static class SlideUntil extends ActionBase {
        TelescopicArmSubsystem telescopicArmSubsystem;
        double wantedPos;
        int direction;

        public SlideUntil(TelescopicArmSubsystem telescopicArmSubsystem, double posInCm) {
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            wantedPos = posInCm * telescopicArmSubsystem.ticksPerCm;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void initialize() {
            direction = wantedPos < telescopicArmSubsystem.getPosition() ? 1 : -1;
            telescopicArmSubsystem.setArmPower(direction);
        }

        @Override
        public boolean isFinished() {
            return telescopicArmSubsystem.getPosition() * direction > wantedPos * direction;
        }
    }
    public static class GoTo extends ActionBase{
        TelescopicArmSubsystem telescopicArmSubsystem;
        double wantedPos;
        final double kp = 0.0069;
        public GoTo(TelescopicArmSubsystem telescopicArmSubsystem, double posInCm){
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            wantedPos = posInCm * telescopicArmSubsystem.ticksPerCm;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void execute() {
            double power = (wantedPos - telescopicArmSubsystem.getPosition()) * kp;
            telescopicArmSubsystem.setArmPower(power);
        }

        @Override
        public boolean isFinished() {
            return wantedPos - telescopicArmSubsystem.getPosition() < 16.473;
        }
    }
    public static class GoHome extends ActionBase {
        TelescopicArmSubsystem telescopicArmSubsystem;
        final double requiredCurrent = 3;
        public GoHome(TelescopicArmSubsystem telescopicArmSubsystem) {
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void initialize(){
            telescopicArmSubsystem.setArmPower(-1);
        }

        @Override
        public boolean isFinished() {
            return telescopicArmSubsystem.getCurrent() > requiredCurrent || telescopicArmSubsystem.getPosition() < 10;
        }
    }

    public static class ExtensionManual extends ActionBase{
        TelescopicArmSubsystem telescopicArmSubsystem;
        Supplier<Double> power;
        public ExtensionManual(TelescopicArmSubsystem telescopicArmSubsystem, Supplier<Double> power){
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            this.power = power;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void execute() {
            telescopicArmSubsystem.setArmPower(power.get());
        }
    }


}
