package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

@TeleOp(group = "tests")
public class PowerTakeOffTester extends ActionOpMode {
    DcMotorEx rf, rb, lf, lb;
    GamepadEx gamepad;

    @Override
    public void initialize() {
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        lb = hardwareMap.get(DcMotorEx.class, "leftBack");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void run() {
        rf.setPower(gamepad.getRightX());
        rb.setPower(-gamepad.getRightX());
        lf.setPower(gamepad.getRightX());
        lb.setPower(-gamepad.getRightX());
        multipleTelemetry.addData("rf pos", rf.getCurrentPosition());
        multipleTelemetry.addData("rb pos", rb.getCurrentPosition());
        multipleTelemetry.addData("lf pos", lf.getCurrentPosition());
        multipleTelemetry.addData("lb pos", lb.getCurrentPosition());
        multipleTelemetry.update();
        super.run();
    }
}
