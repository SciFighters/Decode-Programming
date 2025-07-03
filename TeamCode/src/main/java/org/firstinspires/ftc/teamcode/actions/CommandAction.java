package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class CommandAction implements Action {
    CommandBase command;
    boolean initialized = false;
    public CommandAction(CommandBase command){
        this.command = command;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!initialized) {
            command.initialize();
            initialized = true;
        }
        command.execute();
        boolean isFinished = command.isFinished();
        if(isFinished){
            command.end(false);
        }
        return !isFinished;
    }

}
