package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class ActionCmd extends ActionBase{
    @Override
    public boolean isFinished() {
        return !run(new TelemetryPacket());
    }
}
