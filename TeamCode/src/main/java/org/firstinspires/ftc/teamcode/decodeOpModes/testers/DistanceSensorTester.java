package org.firstinspires.ftc.teamcode.decodeOpModes.testers;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class DistanceSensorTester extends ActionOpMode {
    Rev2mDistanceSensor right;
    Rev2mDistanceSensor left;
    ElapsedTime time;
    double lastTime, currentTime, lastIn;
    int i = 0;
    int count = 0;
    double leftDistance = 0, rightDistance = 0;
    double start = 0, fStart = 0;
    IntakeSubsystem intakeSubsystem;
    MecanumDrive mecanumDrive;

    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(hardwareMap,new Pose2d( -63,0,-Math.PI));
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        left = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance");
        right = hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance");
        time = new ElapsedTime();
        currentTime = time.seconds();
        schedule(new MecanumCommands.Drive(mecanumDrive,() -> (double)gamepad2.left_stick_x, () -> -(double)gamepad2.left_stick_y,() -> (double)gamepad2.right_stick_x));
//        new LynxController()
    }

    @Override
    public void run() {
        i++;
        if(count < 3){
            intakeSubsystem.setPower(gamepad1.left_stick_x);
            start = time.seconds();
            fStart = start;
        } else if (count == 3) {
            start = time.seconds();
            if(start - fStart > 0.3){
                intakeSubsystem.setPower(0);
            }
        }else
        {
            intakeSubsystem.setPower(-0.5);
            if(time.seconds() - start > 0.7){
                count = 3;
            }
        }
        lastTime = currentTime;
        currentTime = time.seconds();
        if (i % 10 == 0) {
            double lastLeft = leftDistance;
            double lastRight = rightDistance;
            leftDistance = left.getDistance(DistanceUnit.CM);
            rightDistance = right.getDistance(DistanceUnit.CM);
            if (leftDistance < 5 && lastLeft > 5) {
                count += 1;
            }
            if (rightDistance < 5 && lastRight > 5) {
                count += 1;
            }
            if (gamepad1.a) {
                count = 0;
            }
            multipleTelemetry.addData("leftDist", Math.min(40, leftDistance));
            multipleTelemetry.addData("rightDist", Math.min(40, rightDistance));
            multipleTelemetry.addData("loopTime", 1 / (currentTime - lastTime));
            multipleTelemetry.addData("threshold", 5);
            multipleTelemetry.addData("count", count);
            multipleTelemetry.update();
        }

        super.run();
    }
}

