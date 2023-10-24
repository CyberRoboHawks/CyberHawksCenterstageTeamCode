package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AutonomousBase extends LinearOpMode {
    static final float DRIVE_SPEED = 0.2f;
    private final ElapsedTime runtime = new ElapsedTime();
    Commands commands = new Commands();
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    AprilTagProcessor tagProcessor = null;
    private WebcamName webcam1, webcam2;
    VisionPortal visionPortal = null;
    public void startupInit() {
        commands.init(hardwareMap);
        robot.init(hardwareMap);

        if (robot.hasCamera) {
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .build();
                visionPortal = new VisionPortal.Builder()
                        .addProcessor(tagProcessor)
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .setCameraResolution(new Size(1920, 1080))
                        .build();
           // }

        }
        telemetry.addData("Status:", "ready");
        commands.printRobotStatus(telemetry);
        telemetry.update();
    }

    protected void executeOperations(CenterStageEnums.TapeColor color) throws InterruptedException {
        runtime.reset();

        // Identify pixel/team element placement

        // Orient to tape line

        // Find tape line

        // Orient for pixel placement
        commands.driveForward(.2, 5, 3, telemetry);

        // Deliver ground pixel
        commands.deliverGroundPixel();
        sleep(250);
        commands.driveBackwards(.2, 2, 3, telemetry);

        // Orient towards backdrop red/blue
        if (color == CenterStageEnums.TapeColor.Red){
            commands.spinRight(.3, -90, 5);
        }else {
            commands.spinLeft(.3, 90, 5);
        }


        commands.driveForward(.2, 4, 3, telemetry);
        runtime.reset();
        // Drive towards backdrop find April tag
        while (!isStopRequested() && robot.hasCamera && runtime.seconds() < 5) {
            int targetId = 4;
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Rotate, targetId, telemetry);
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Strafe, targetId, telemetry);
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Straight, targetId, telemetry);
        }

        // RoboHawks - spin 180 degrees before deliver pixel on backdrop
        if (robot.isRoboHawks){
            commands.spinLeft(.3, 90, 6);
            sleep(250);
            commands.reverseDriveMotorDirection();
            sleep(250);
            commands.approachBackdrop(25, 10, telemetry);
            sleep(250);
        }

        // Cyberhawks - deliver pixel on backdrop
        if (!robot.isRoboHawks) {
            commands.setArmPosition(CenterStageEnums.ArmDirection.Up, 3, telemetry);
            sleep(250);
            commands.approachBackdrop(4, 10, telemetry);
            sleep(250);
            commands.setGrabberPosition(0.6);  // Open grabber
            sleep(250);

            commands.setArmPosition(CenterStageEnums.ArmDirection.Down, 3, telemetry);
            sleep(250);
            commands.driveBackwards(DRIVE_SPEED, 5, 2, telemetry);
        }

        // Park

    }
}