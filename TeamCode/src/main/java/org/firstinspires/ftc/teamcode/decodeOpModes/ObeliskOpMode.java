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

    public abstract void mecanumInit();

    @Override
    public void initialize() {
        mecanumInit();
        limelightSubsystem = new LimelightSubsystem(hardwareMap, TeamColor.RED, mecanumDrive);//for default

    }

    @Override
    public void initialize_loop() {
        motif = (limelightSubsystem.getMotif() != null)? limelightSubsystem.getMotif() : motif;
    }

}
