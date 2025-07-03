package org.firstinspires.ftc.teamcode.actions;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class TeleAuto extends ActionOpMode {
    TrajectoryActionBuilder moveSamplesToObs,Turn0,Turn90;
    MecanumDrive drive;
    Pose2d pose2d;
    GamepadEx driverGamepad;
    GamepadEx systemGamepad;
    Button systemA, driverA;
    Button systemB, driverB;
    Button systemY, driverY;
    Button systemX, driverX;
    Button systemDPadDown, driverDPadDown;
    Button systemDPadUp, driverDPadUp;
    Button systemDPadRight, driverDPadRight;
    Button systemDPadLeft, driverDPadLeft;
    Button systemRightBumper, driverRightBumper;
    Button systemLeftBumper, driverLeftBumper;
    Button driverStart;
    Button systemBack, driverBack;
    Button systemStart;
    Button systemLeftStick, systemRightStick;
    Button driverLeftStick, driverRightStick;
    @Override
    public void initialize() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0, -64), Rotation2d.fromDouble(0)));
        moveSamplesToObs = drive.actionBuilder(drive.localizer.getPose())

                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(12, -44), 0)
                .splineToSplineHeading(new Pose2d(32, -28, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(41, -19), 0)

                .splineToConstantHeading(new Vector2d(46.5, -48), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(52, -23), 0)
                .splineToConstantHeading(new Vector2d(57.5, -48), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(60, -23), 0)
                .splineToConstantHeading(new Vector2d(64, -48), -Math.PI / 2);
        Turn0 = drive.actionBuilder(drive.localizer.getPose()).turnTo(0);
        Turn90 = drive.actionBuilder(drive.localizer.getPose()).turnTo(Math.PI/2);


    }

    @Override
    public void run() {
        super.run();
        drive.updatePoseEstimate();
        pose2d = drive.localizer.getPose();
        if(gamepad1.a && runningActions.size() <= 1){
            runningActions.add(moveSamplesToObs.build());
        }

    }

}
