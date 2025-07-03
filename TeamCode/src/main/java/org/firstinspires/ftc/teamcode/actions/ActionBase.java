package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class ActionBase extends CommandBase implements Action {
    boolean initialized = false;
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!initialized){
            initialize();
            initialized = true;
        }
        execute();
        boolean finished = isFinished();
        if(isFinished()){
            end(false);
        }
        return !finished;
    }
}
