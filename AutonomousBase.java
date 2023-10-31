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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

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

    protected void executeOperations(TapeColor color, CenterStageEnums.StrafeDirection parkingDirection) throws InterruptedException {
        if (robot.hasBlinkin) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        }

        runtime.reset();
        telemetry.setAutoClear(false);
        telemetry.addData("Color: ", color);

        // Identify pixel/team element placement
        TapeLocation tapeLocation = TapeLocation.Right;

        telemetry.addData("Tape location: ", tapeLocation);
        AprilTag aprilTag = GetTagID(color, tapeLocation);
        telemetry.addData("April tag: ", aprilTag);
        telemetry.update();

        // Move off the start
        commands.driveForward(DRIVE_SPEED, 26, 3);

        // Turn to tape line
        TurnTowardsSpikeMark(color, tapeLocation);

        // Find tape line
        boolean isStrafe = (tapeLocation == TapeLocation.Left);
        commands.ApproachTape(color, isStrafe, 3);

        // Orient for pixel placement
        if (tapeLocation != TapeLocation.Left)
            commands.driveBackwards(DRIVE_SPEED_FAST, 3, 3);
        else if (tapeLocation == TapeLocation.Left)
            commands.driveBackwards(DRIVE_SPEED_FAST, 2, 3);

        // Deliver ground pixel
        commands.deliverSpikeMarkPixel();

        sleep(250);
        if (tapeLocation == TapeLocation.Center) {
            commands.driveBackwards(DRIVE_SPEED_FAST, 4, 3);
        }
        if (tapeLocation == TapeLocation.Left) {
            commands.strafeRight(DRIVE_SPEED_FAST, 9, 3);
        }

        // Orient towards backdrop red/blue
        if (color == TapeColor.Red && tapeLocation == TapeLocation.Center ||
                color == TapeColor.Red && tapeLocation == TapeLocation.Left) {
            commands.spinRight(ROTATE_SPEED, -90, 5);
            commands.strafeLeft(DRIVE_SPEED_FAST, 2, 2);
        } else if (color == TapeColor.Blue && tapeLocation == TapeLocation.Center) {
            commands.spinLeft(ROTATE_SPEED, 90, 5);
            commands.strafeRight(DRIVE_SPEED_FAST, 2, 2);
        } else if (color == TapeColor.Blue && tapeLocation == TapeLocation.Right) {
            commands.driveBackwards(DRIVE_SPEED_FAST, 5, 3);
            commands.spinLeft(ROTATE_SPEED, 90, 5);
        } else if (color == TapeColor.Blue && tapeLocation == TapeLocation.Left) {
            commands.spinLeft(ROTATE_SPEED, 90, 5);
        }
        telemetry.update();

    //        if (tapeLocation == TapeLocation.Center) {
    //            // Center red reorientation
    //            commands.driveForward(DRIVE_SPEED_FAST, 6, 3);
    //            commands.strafeLeft(DRIVE_SPEED_FAST, 3, 2);
    //        }

        runtime.reset();
        boolean tagsFound = false;
        while (!isStopRequested() && robot.hasCamera && runtime.seconds() < 3 && !tagsFound) {
            tagsFound = GetCloserToAprilTags(color, 6);
            sleep(250);
        }

        runtime.reset();
        // Drive towards backdrop find April tag
        while (!isStopRequested() && robot.hasCamera && runtime.seconds() < 6) {
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
            commands.approachBackdrop(6, 10);
            sleep(250);
            commands.setGrabberPosition(robot.GRABBER_OPEN);
            sleep(250);
            commands.setArmPosition(CenterStageEnums.ArmDirection.Down, 3);
            sleep(250);
            commands.driveBackwards(DRIVE_SPEED_FAST, 4, 2);
        }

        // Park
        if (parkingDirection == Left)
            commands.strafeLeft(DRIVE_SPEED_FAST, GetStrafeDistance(parkingDirection, aprilTag), 5);
        if (parkingDirection == Right)
            commands.strafeRight(DRIVE_SPEED_FAST, GetStrafeDistance(parkingDirection, aprilTag), 5);
        
        commands.driveForward(DRIVE_SPEED_FAST, 8, 2);
    }

    private boolean GetCloserToAprilTags(TapeColor color, int distance) {
        ArrayList<AprilTagDetection> detections = tagProcessor.getFreshDetections();
        boolean tagsFound = detections.size() > 0;
        telemetry.addData("detections: ", detections.size());
        telemetry.update();
        if (!tagsFound)
            commands.driveForward(DRIVE_SPEED_FAST, distance, 3);

        if (color == TapeColor.Blue){
            if (commands.getAngle() > 90) commands.spinRight(ROTATE_SPEED, 90, 2);
            else if (commands.getAngle() < 90) commands.spinLeft(ROTATE_SPEED, 90, 2);
        }
        if (color == TapeColor.Red){
            if (commands.getAngle() > -90) commands.spinRight(ROTATE_SPEED, -90, 2);
            else if (commands.getAngle() < -90) commands.spinLeft(ROTATE_SPEED, -90, 2);
        }
        return  tagsFound;
    }

    private void TurnTowardsSpikeMark(TapeColor color, TapeLocation tapeLocation) {
        if (color == TapeColor.Red) {
            if (tapeLocation == TapeLocation.Left) {
                commands.strafeLeft(.3, 6, 2);
            }
            if (tapeLocation == TapeLocation.Right) {
                commands.spinRight(ROTATE_SPEED, -90, 3);
            }
        }
        if (color == TapeColor.Blue) {
            if (tapeLocation == TapeLocation.Left) {
                commands.strafeLeft(.3, 6, 2);
            }
            if (tapeLocation == TapeLocation.Right) {
                commands.spinRight(ROTATE_SPEED, -90, 3);
            }
        }
    }

    private int GetStrafeDistance(CenterStageEnums.StrafeDirection direction, AprilTag aprilTag) {
        int distance = 26;
        if ((direction == Right && (aprilTag == AprilTag.RedRight || aprilTag == AprilTag.BlueRight)) ||
                (direction == Left && (aprilTag == AprilTag.RedLeft || aprilTag == AprilTag.BlueLeft)))
            distance = 18;

        if ((direction == Right && (aprilTag == AprilTag.RedLeft || aprilTag == AprilTag.BlueLeft)) ||
            (direction == Left && (aprilTag == AprilTag.RedRight || aprilTag == AprilTag.BlueRight)))
            distance = 32;

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