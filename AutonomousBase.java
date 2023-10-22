package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutonomousBase extends LinearOpMode {
    static final float DRIVE_SPEED = 0.2f;

    Commands commands = new Commands();
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping

    public void startupInit() {
        commands.init(hardwareMap);
        robot.init(hardwareMap);

        telemetry.addData("Status:", "ready");
        commands.printRobotStatus(telemetry);
        telemetry.update();
    }

    protected void executeOperations(CenterStageEnums.TapeColor color) throws InterruptedException {
        // drive forward to move off the wall
        if (!robot.hasDriveMotors) {
            telemetry.addData("missing drive motors", "");
            return;
        }

        commands.driveForward(DRIVE_SPEED, 5, 2, telemetry);

        // Identify pixel/team element placement

        // Orient to tape line

        // Find tape line

        // Orient for pixel placement

        // Deliver ground pixel

        // Orient towards backdrop red/blue

        // Drive towards backdrop find April tag

        //

        commands.spinRight(DRIVE_SPEED, -90, 3);
        commands.spinLeft(DRIVE_SPEED, 90, 3);

        commands.driveForward(DRIVE_SPEED, 8.5, 3, telemetry);
        commands.quickSpin(DRIVE_SPEED, -180, 3);
    }
}