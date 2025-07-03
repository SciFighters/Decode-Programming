package org.firstinspires.ftc.teamcode.needle.commands;

import org.firstinspires.ftc.teamcode.actions.ActionBase;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;

public class ClawCommands{
    public static class SetAxisServo extends ActionBase {
        ClawSubsystem clawSubsystem;
        double pos;
        public SetAxisServo(ClawSubsystem clawSubsystem, double pos){
            this.clawSubsystem = clawSubsystem;
            this.pos = pos;
        }

        @Override
        public void initialize() {
            clawSubsystem.setAxisServo(pos);
        }
    }

    public static class SetClampPos extends ActionBase{
        ClawSubsystem clawSubsystem;
        double pos;
        public SetClampPos(ClawSubsystem clawSubsystem, double pos){
            this.clawSubsystem = clawSubsystem;
            this.pos = pos;
        }

        @Override
        public void initialize() {
            clawSubsystem.setClampServo(pos);
        }
    }
}
