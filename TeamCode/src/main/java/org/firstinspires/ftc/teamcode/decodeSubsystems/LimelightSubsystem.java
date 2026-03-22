package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter.TeamColor;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    public final double metersToInch = 39.3700787;
    public TeamColor color;
    public Vector2d initialLimelightPos = new Vector2d(1.5748, 0);
    public Vector2d limelightByTurret = new Vector2d(-5.6868, 0);//y: 2.6454415267717
    public Pose2d aprilTagPos;
    public MecanumDrive mecanumDrive;
    int pipeline = 1;
    private final Limelight3A limelight;
    public double currentTx;

    @Override
    public void periodic() {
        super.periodic();
        currentTx = getTx();
    }

    public LimelightSubsystem(HardwareMap hm, TeamColor color, MecanumDrive mecanumDrive) {
        limelight = hm.get(Limelight3A.class, "limelight");
        setPipeline((color == TeamColor.RED) ? 1 : 2);
        startLimelight();
        this.color = color;
        this.mecanumDrive = mecanumDrive;
        aprilTagPos = color == TeamColor.RED ? new Pose2d(-58, 56, Rotation2d.fromDegrees(-54)) : new Pose2d(-58, -56, Rotation2d.fromDegrees(54));
    }

    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }


    public void startLimelight() {
        limelight.start();
    }

    public void stopLimelight() {
        limelight.stop();
    }

    public void updateLimelightDegree(double angle) {
        limelight.updateRobotOrientation(angle);
    }

    public TeamColor getTeamColor() {
        TeamColor goalColor = getColorFromGoal();
        if (goalColor != null) {
            return goalColor;
        }
        return getColorFromObelisk();
    }

    public TeamColor getColorFromObelisk() {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            int id = fiducialResult.getFiducialId();

            if (21 <= id && id <= 23) {
                if (fiducialResult.getCameraPoseTargetSpace().getPosition().x > 0) {
                    return TeamColor.RED;
                }
                return TeamColor.BLUE;
            }
        }
        return null;
    }

    public TeamColor getColorFromGoal() {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            int id = fiducialResult.getFiducialId();

            if (id == 20 || id == 24) {
                if (fiducialResult.getRobotPoseFieldSpace().getPosition().y > 0) {
                    return TeamColor.RED;
                }
                return TeamColor.BLUE;
            }
        }
        return null;
    }

    public void setColor(TeamColor teamColor) {
        aprilTagPos = teamColor == TeamColor.RED ? new Pose2d(-58, 56, Rotation2d.fromDegrees(-54)) : new Pose2d(-58, -56, Rotation2d.fromDegrees(54));
    }

    public int getMotif() {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            int id = fiducialResult.getFiducialId();
            switch (id) {
                case 21:
                    return 0;
                case 22:
                    return 1;
                case 23:
                    return 2;
            }
        }
        return -1;
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

    public Position getLimelightByTagPos() {
        Position pos1 = new Position();
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {

            switch (color) {
                case RED:
                    pos1 = fiducialResult.getFiducialId() == 24 ?
                            fiducialResult.getRobotPoseFieldSpace().getPosition()
                            : pos1;
                    break;
                case BLUE:
                    pos1 = fiducialResult.getFiducialId() == 20 ?
                            fiducialResult.getRobotPoseFieldSpace().getPosition()
                            : pos1;
                    break;
            }
        }
        return pos1;

    }

    public Position getLimelightByTagPosMT2() {
        return limelight.getLatestResult().getBotpose_MT2().getPosition();
    }

    //given the limelight pos, returns if the pos is valid
    public boolean checkLimelightResults(Vector2d robotPos) {
        return (Math.abs(mecanumDrive.localizer.getPose().position.y - robotPos.getY()) < 3 && Math.abs(mecanumDrive.localizer.getPose().position.y - robotPos.getY()) < 3);
    }

    public Position getRobotPos(double turretHeading) {

        double robotHeading = mecanumDrive.localizer.getPose().heading.toDouble();
        if (getGoalID()) {
            Position limelight3d = getLimelightByTagPos();
            Vector2d limelightPos = initialLimelightPos.plus(limelightByTurret.rotateBy(turretHeading)).rotateBy(robotHeading * 180 / Math.PI);
            Vector2d limelightByTag = new Vector2d(limelight3d.x * metersToInch, limelight3d.y * metersToInch);
            Vector2d result = limelightByTag.minus(limelightPos);
            return new Position(DistanceUnit.INCH, result.getX(), result.getY(), limelight3d.z, 0);
        }
        return null; // only if result isn't in field and or is invalid
    }

    public Position getRobotPosMT2(double turretHeading) {

        double robotHeading = mecanumDrive.localizer.getPose().heading.toDouble();
        if (getGoalID()) {
            double turretFieldAngle = robotHeading * 180 / Math.PI + turretHeading;
            if (turretFieldAngle > 180) {
                turretFieldAngle -= 360;
            } else if (turretFieldAngle < -180) {
                turretFieldAngle += 360;
            }
            limelight.updateRobotOrientation(turretFieldAngle);
            Position limelight3d = getLimelightByTagPosMT2();
            Vector2d limelightPos = initialLimelightPos.plus(limelightByTurret.rotateBy(turretHeading)).rotateBy(robotHeading * 180 / Math.PI);
            Vector2d limelightByTag = new Vector2d(limelight3d.x * metersToInch, limelight3d.y * metersToInch);
            Vector2d result = limelightByTag.minus(limelightPos);
            return new Position(DistanceUnit.INCH, result.getX(), result.getY(), limelight3d.z, 0);
//            return limelight3d;
        }
        return null;
    }

    public double getTx() {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            switch (SavedValues.teamColor) {
                case RED:
                    if (fiducialResult.getFiducialId() == 24) {
                        return fiducialResult.getTargetXDegrees();
                    }
                case BLUE:
                    if (fiducialResult.getFiducialId() == 20) {
                        return fiducialResult.getTargetXDegrees();
                    }
            }
        }
        return 0;
    }

    public Vector2d getPixelError() {
        List<LLResultTypes.FiducialResult> results = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducialResult : results) {
            switch (color) {
                case RED:
                    if (fiducialResult.getFiducialId() == 24) {
                        return new Vector2d(fiducialResult.getTargetXPixels(), fiducialResult.getTargetYPixels());
                    }
                    break;
                case BLUE:
                    if (fiducialResult.getFiducialId() == 20) {
                        return new Vector2d(fiducialResult.getTargetXPixels(), fiducialResult.getTargetYPixels());
                    }
                    break;
            }
        }
        return null;
    }

}