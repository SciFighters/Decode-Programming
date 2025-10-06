package org.firstinspires.ftc.teamcode.decodeOpModes;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter.TeamColor;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

public abstract class ObeliskOpMode extends ActionOpMode {
    LimelightSubsystem limelightSubsystem;
    MecanumDrive mecanumDrive;
    Motif motif;
    TeamColor teamColor;


    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap, TeamColor.RED, mecanumDrive);//for default
    }

    @Override
    public void initialize_loop() {
        motif = (limelightSubsystem.getMotif() != null)? limelightSubsystem.getMotif() : motif;
        TeamColor teamColor1 = limelightSubsystem.getColorFromObelisk();
        if(teamColor1 != null){
            teamColor = teamColor1;
            limelightSubsystem.setColor(teamColor1);
        }
    }


}
