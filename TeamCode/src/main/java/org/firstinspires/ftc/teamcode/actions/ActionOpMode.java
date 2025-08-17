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
    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(dashTelemetry,telemetry);
}
