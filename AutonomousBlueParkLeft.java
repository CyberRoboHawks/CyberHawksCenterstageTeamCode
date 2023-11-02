package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue - Park Left", group = "Pushbot", preselectTeleOp="TeleOpDrive")
public class AutonomousBlueParkLeft extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        startupInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            executeOperations(CenterStageEnums.TapeColor.Blue, CenterStageEnums.StrafeDirection.Left);
            sleep(30000);
        }
    }
}