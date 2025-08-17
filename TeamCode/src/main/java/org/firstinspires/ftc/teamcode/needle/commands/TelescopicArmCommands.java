package org.firstinspires.ftc.teamcode.needle.commands;


import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

import java.util.function.Supplier;

public class TelescopicArmCommands {
    public static class SlideUntil extends CommandBase {
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
    public static class GoTo extends CommandBase{
        TelescopicArmSubsystem telescopicArmSubsystem;
        double wantedPos;
        final double kp = 0.07, kf = 0.1;
        public GoTo(TelescopicArmSubsystem telescopicArmSubsystem, double posInCm){
            this.telescopicArmSubsystem = telescopicArmSubsystem;
            wantedPos = posInCm;
            addRequirements(telescopicArmSubsystem);
        }

        @Override
        public void execute() {
            double power = (wantedPos - telescopicArmSubsystem.getPosition()) * kp;
            telescopicArmSubsystem.setArmPower(power + kf);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(wantedPos - telescopicArmSubsystem.getPosition()) < 2;
        }
    }
    public static class GoHome extends CommandBase {
        TelescopicArmSubsystem telescopicArmSubsystem;
        final double requiredCurrent = 10;
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
            return telescopicArmSubsystem.getCurrent() > requiredCurrent || telescopicArmSubsystem.getPosition() < 31;
        }
    }

    public static class ExtensionManual extends CommandBase{
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
