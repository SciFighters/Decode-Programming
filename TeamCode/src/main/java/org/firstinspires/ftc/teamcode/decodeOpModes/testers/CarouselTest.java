package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

@TeleOp(name = "CarouselTest", group = "tests")
public class CarouselTest extends ActionOpMode {
    CarouselSubsystem carouselSubsystem;
    GamepadEx gamepadEx;
    Button A, B;

    @Override
    public void initialize() {
        gamepadEx = new GamepadEx(gamepad1);

        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        // Create commands
        CarouselCommands.MoveToPos moveToPos = new CarouselCommands.MoveToPos(carouselSubsystem, 1000);
        CarouselCommands.ThirdOfSpin thirdOfSpin = new CarouselCommands.ThirdOfSpin(carouselSubsystem);
        CarouselCommands.SortByMotif sortByMotif = new CarouselCommands.SortByMotif(Motif.GPP, carouselSubsystem);
        CarouselCommands.Discharge discharge = new CarouselCommands.Discharge(carouselSubsystem);

        // sequential command group
        SequentialCommandGroup fullSequence = new SequentialCommandGroup(
                moveToPos, new WaitCommand(2000),
                thirdOfSpin, new WaitCommand(2000),
                sortByMotif, new WaitCommand(2000),
                discharge
        );
        A = gamepadEx.getGamepadButton(GamepadKeys.Button.A);
        B = gamepadEx.getGamepadButton(GamepadKeys.Button.B);
//        A.whenPressed(new CarouselCommands.SmartDischarge(carouselSubsystem));
        B.whenPressed(new CarouselCommands.SortByMotif(Motif.PGP,carouselSubsystem));
    }

    @Override
    public void run() {
        multipleTelemetry.addData("pos", carouselSubsystem.getPosition());
        multipleTelemetry.addData("angle", carouselSubsystem.getAngle());
        multipleTelemetry.update();
        super.run();
    }
}
