package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

public class LimelightSubsystems extends SubsystemBase {
    private Limelight3A limelight;
    int pipeline = 1;
    public AutoShooter.TeamColor color;
    public Vector2d initialLimelightPos = new Vector2d(0, 2); //TODO:change
    public Vector2d limelightByTurret = new Vector2d(0, 2.6454415267717);
    public Vector2d aprilTagPos;
    public MecanumDrive mecanumDrive;

    public LimelightSubsystems(HardwareMap hm, AutoShooter.TeamColor color, MecanumDrive mecanumDrive) {
        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        this.color = color;
        this.mecanumDrive = mecanumDrive;
        aprilTagPos = color == AutoShooter.TeamColor.RED ? new Vector2d(-58, 56) : new Vector2d(-58, -56);
    }

    public Motif getMotif() {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            int id = fiducialResult.getFiducialId();
            switch (id) {
                case 21:
                    return Motif.GPP;
                case 22:
                    return Motif.PGP;
                case 23:
                    return Motif.PPG;
            }
        }
        return null;
    }

    public boolean getGoalID() {
        boolean id = false;
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            switch (color) {
                case RED:
                    id = fiducialResult.getFiducialId() == 24 ? true : id;
                    break;
                case BLUE:
                    id = fiducialResult.getFiducialId() == 20 ? true : id;
                    break;
            }
        }
        return id;
    }

    public Vector2d getLimelightByTagPos() {
        Vector2d pos = new Vector2d(0, 0);
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();

        for (LLResultTypes.FiducialResult fiducialResult : results) {
            switch (color) {
                case RED:
                    pos = fiducialResult.getFiducialId() == 24 ?
                            new Vector2d(fiducialResult.getCameraPoseTargetSpace().getPosition().x, fiducialResult.getCameraPoseTargetSpace().getPosition().y) : pos;
                    break;
                case BLUE:
                    pos = fiducialResult.getFiducialId() == 20 ?
                            new Vector2d(fiducialResult.getCameraPoseTargetSpace().getPosition().x, fiducialResult.getCameraPoseTargetSpace().getPosition().y) : pos;
                    break;
            }
        }
        return pos;
    }
    //given the limelight pos, returns if the pos is valid
    public boolean checkLimelightResults(Vector2d robotPos) {
        return (Math.abs(mecanumDrive.localizer.getPose().position.y - robotPos.getY()) < 3 && Math.abs(mecanumDrive.localizer.getPose().position.y - robotPos.getY()) < 3);
    }

    public Vector2d getRobotPos(double robotHeading, double turretHeading) {
        if (getGoalID()) {
            Vector2d limelightPos = initialLimelightPos.plus(limelightByTurret.rotateBy(turretHeading)).rotateBy(robotHeading);
            Vector2d limelightByTag = getLimelightByTagPos();
            return limelightByTag.minus(limelightPos).plus(aprilTagPos);
        }
        return new Vector2d(1000, 1000); // only if result isn't in field and or is invalid
    }
}