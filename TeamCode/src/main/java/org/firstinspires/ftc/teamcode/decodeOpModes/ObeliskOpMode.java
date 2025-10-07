package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter.TeamColor;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

public abstract class ObeliskOpMode extends ActionOpMode {
    public LimelightSubsystem limelightSubsystem;
    public MecanumDrive mecanumDrive;
    public Motif motif;
    public TeamColor teamColor;



    @Override
    public void initialize_loop() {
        motif = (limelightSubsystem.getMotif() != null) ? limelightSubsystem.getMotif() : motif;
        TeamColor teamColor1 = limelightSubsystem.getColorFromObelisk();
        if (teamColor1 != null) {
            teamColor = teamColor1;
            limelightSubsystem.setColor(teamColor1);
        }
    }
    public void runInit(){}

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        try {
            while (opModeInInit()) {
                initialize_loop();
            }
            runInit();
            while (!isStopRequested() && opModeIsActive()) {
                run();
            }
        } finally {
            try {
                end();
            } finally {
                reset();
            }
        }
    }
}
