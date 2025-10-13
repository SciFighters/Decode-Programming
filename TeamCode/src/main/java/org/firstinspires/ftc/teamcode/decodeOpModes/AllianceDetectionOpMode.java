package org.firstinspires.ftc.teamcode.decodeOpModes;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter.TeamColor;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

public abstract class AllianceDetectionOpMode extends ActionOpMode {
    public LimelightSubsystem limelightSubsystem;
    public MecanumDrive mecanumDrive;
    public Motif motif;
    public TeamColor teamColor;



    @Override
    public void initialize_loop() {
        motif = (limelightSubsystem.getMotif() != null) ? limelightSubsystem.getMotif() : motif;
        TeamColor teamColor1 = limelightSubsystem.getTeamColor();
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
