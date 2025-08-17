package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//made while teaching
public class Hob extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor RightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()){
            leftMotor.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
            RightMotor.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);

            telemetry.addData("maybe the right one", leftMotor.getPower());
            telemetry.addData("maybe not the right one", RightMotor.getPower());
            telemetry.update();
        }
    }
}
