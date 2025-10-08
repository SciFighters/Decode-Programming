package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;

@TeleOp(name = "CarouselTest")
public class CarouselTest extends ActionOpMode {
    CarouselSubsystem carouselSubsystem;

    @Override
    public void initialize() {
        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        // Create commands
        CarouselCommands.MoveToPos moveToPos = new CarouselCommands.MoveToPos(carouselSubsystem, 1000);
        CarouselCommands.ThirdOfSpin thirdOfSpin = new CarouselCommands.ThirdOfSpin(carouselSubsystem);
        CarouselCommands.SortByMotif sortByMotif = new CarouselCommands.SortByMotif(Motif.GPP, 1, carouselSubsystem);
        CarouselCommands.Discharge discharge = new CarouselCommands.Discharge(carouselSubsystem);

        // Combine them into a sequential command group
        SequentialCommandGroup fullSequence = new SequentialCommandGroup(
                moveToPos,
                new WaitCommand(2000),
                thirdOfSpin, new WaitCommand(2000),
                sortByMotif, new WaitCommand(2000),
                discharge
        );

        // Schedule once; ActionOpMode automatically runs the scheduler
        CommandScheduler.getInstance().schedule(fullSequence);
    }

    @Override
    public void run() {
        multipleTelemetry.addData("pos", carouselSubsystem.getPosition());
        multipleTelemetry.update();
        super.run();
    }
}
