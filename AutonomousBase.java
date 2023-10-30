package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CenterStageEnums.StrafeDirection.Left;
import static org.firstinspires.ftc.teamcode.CenterStageEnums.StrafeDirection.Right;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStageEnums.AprilTag;
import org.firstinspires.ftc.teamcode.CenterStageEnums.TapeColor;
import org.firstinspires.ftc.teamcode.CenterStageEnums.TapeLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AutonomousBase extends LinearOpMode {
    static final float DRIVE_SPEED = 0.3f;
    static final float DRIVE_SPEED_FAST = 0.5f;

    static final float ROTATE_SPEED = 0.3f;

    private final ElapsedTime runtime = new ElapsedTime();
    Commands commands = new Commands(telemetry);
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    AprilTagProcessor tagProcessor = null;
    VisionPortal visionPortal = null;
    double startingAngle;

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
        }
        telemetry.addData("Status:", "ready");
        startingAngle = commands.getAngle();
        telemetry.addData("angle", startingAngle);
//        commands.printRobotStatus(telemetry);
       telemetry.update();
    }

    protected void executeOperations(TapeColor color) throws InterruptedException {
        if (robot.hasBlinkin){
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        }

        runtime.reset();
        telemetry.setAutoClear(false);
        telemetry.addData("Color: ",color);

        // Identify pixel/team element placement
        TapeLocation tapeLocation = TapeLocation.Right;

        telemetry.addData("Tape location: ",tapeLocation);
        AprilTag aprilTag = GetTagID(color, tapeLocation);
        telemetry.addData("April tag: ",aprilTag);
        telemetry.update();

        // Orient to tape line
        commands.driveForward(DRIVE_SPEED, 26, 3);

        if (color == TapeColor.Red){
            if (tapeLocation == TapeLocation.Left) {
                commands.strafeLeft(.3, 6, 2);
            }
            if (tapeLocation == TapeLocation.Right) {
                commands.spinRight(ROTATE_SPEED, -90, 3);
                telemetry.addData("Tape location - spin right 90",tapeLocation);
            }
        }

        boolean isStrafe = (color == TapeColor.Red && tapeLocation == TapeLocation.Left);
         // Find tape line
        commands.ApproachTape(color, isStrafe, 3);
        telemetry.addData("Approach tape: ",color);
        telemetry.update();

        // Orient for pixel placement
        if (color == TapeColor.Red && tapeLocation != TapeLocation.Left)
            commands.driveBackwards(DRIVE_SPEED_FAST, 3, 3);
        else if (color == TapeColor.Red && tapeLocation == TapeLocation.Left)
            commands.driveBackwards(DRIVE_SPEED_FAST, 2, 3);

        // Deliver ground pixel
        commands.deliverGroundPixel();


        sleep(250);
        if (tapeLocation == TapeLocation.Center){
            commands.driveBackwards(DRIVE_SPEED_FAST, 4, 3);
        }
        if (color == TapeColor.Red && tapeLocation == TapeLocation.Left ){
            commands.strafeRight(DRIVE_SPEED_FAST,9,3);
        }

        // Orient towards backdrop red/blue
        if (color == TapeColor.Red && tapeLocation == TapeLocation.Center ||
        color == TapeColor.Red && tapeLocation == TapeLocation.Left) {
            commands.spinRight(ROTATE_SPEED, -90, 5);
            commands.strafeLeft(ROTATE_SPEED, 2, 2);
            telemetry.addData("Red - spin right 90","");
        } else if (color == TapeColor.Blue && tapeLocation == TapeLocation.Center){
            commands.spinLeft(ROTATE_SPEED, 90, 5);
            telemetry.addData("Blue - spin left 90","");
        }
        telemetry.update();
sleep(30000);

        if (tapeLocation == TapeLocation.Center){
            // Center red reorientation
            commands.driveForward(DRIVE_SPEED_FAST, 6, 3);
            commands.strafeLeft(DRIVE_SPEED_FAST, 3, 2);
        }


        runtime.reset();
        // Drive towards backdrop find April tag
        while (!isStopRequested() && robot.hasCamera && runtime.seconds() < 6) {
            int targetId = 4;
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Rotate, aprilTag.getValue());
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Strafe, aprilTag.getValue());
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Straight, aprilTag.getValue());
        }

        // RoboHawks - spin 180 degrees before deliver pixel on backdrop
//        if (robot.isRoboHawks){
//            commands.spinLeft(.3, 90, 6);
//            sleep(250);
//            commands.reverseDriveMotorDirection();
//            sleep(250);
//              //Raise arm
//            commands.approachBackdrop(25, 10);
//            sleep(250);
//        }

        // Cyberhawks - deliver pixel on backdrop
        if (!robot.isRoboHawks) {
            commands.setArmPosition(CenterStageEnums.ArmDirection.Up, 3);
            sleep(250);
            commands.approachBackdrop(4, 10);
            sleep(250);
            commands.setGrabberPosition(robot.GRABBER_OPEN);
            sleep(250);
            commands.setArmPosition(CenterStageEnums.ArmDirection.Down, 3);
            sleep(250);
            commands.driveBackwards(DRIVE_SPEED_FAST, 4, 2);
        }

        // Park
        commands.strafeLeft(DRIVE_SPEED_FAST, GetStrafeDistance(Left, aprilTag), 5);
        commands.driveForward(DRIVE_SPEED_FAST, 10, 2);
    }

    private int GetStrafeDistance(CenterStageEnums.StrafeDirection direction, AprilTag aprilTag){
        int distance = 30;
        if (direction == Right && aprilTag == AprilTag.RedRight ||
            direction == Left && aprilTag == AprilTag.RedLeft)
            distance = 18;

        if (direction == Right && aprilTag == AprilTag.RedLeft ||
            direction == Left && aprilTag == AprilTag.RedRight)
            distance = 38;

        return distance;
    }

    private AprilTag GetTagID(TapeColor color, TapeLocation tapeLocation) {
        if (color == TapeColor.Blue) {
            if (tapeLocation == TapeLocation.Left) {
                return AprilTag.BlueLeft;
            } else if (tapeLocation == TapeLocation.Right) {
                return AprilTag.BlueRight;
            } else {
                return AprilTag.BlueCenter;
            }
        } else {  // CenterStageEnums.TapeColor.Red
            if (tapeLocation == TapeLocation.Left) {
                return AprilTag.RedLeft;
            } else if (tapeLocation == TapeLocation.Right) {
                return AprilTag.RedRight;
            } else {
                return AprilTag.RedCenter;
            }
        }
    }
}