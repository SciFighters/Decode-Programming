package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public abstract class ActionOpMode extends CommandOpMode {
    public final FtcDashboard dash = FtcDashboard.getInstance();
    Telemetry dashTelemetry = dash.getTelemetry();

    public List<Action> runningActions = new ArrayList<>();
    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(dashTelemetry,telemetry);

    public void runActions() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void run() {
        super.run();
        runActions();
    }
}
