package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CenterStageEnums.StrafeDirection.Left;
import static org.firstinspires.ftc.teamcode.CenterStageEnums.StrafeDirection.Right;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStageEnums.AprilTag;
import org.firstinspires.ftc.teamcode.CenterStageEnums.TapeColor;
import org.firstinspires.ftc.teamcode.CenterStageEnums.TapeLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public abstract class AutonomousBase extends LinearOpMode {
    static final float DRIVE_SPEED = 0.3f;
    static final float DRIVE_SPEED_FAST = 0.6f;
    private static final String TFOD_MODEL_ASSEST = "CH.tflite";
    private static final String[] LABELS = {
            "Blue Prop",
            "Red Prop"
    };
    private final ElapsedTime runtime = new ElapsedTime();
    private final int CAMERA_WIDTH = 864;
    private final int CAMERA_HEIGHT = 480;
    Commands commands = new Commands(telemetry);
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    AprilTagProcessor tagProcessor = null;
    VisionPortal visionPortal = null;
    double startingAngle;
    double targetDistance = 90;
    private TfodProcessor tfod;

    public void startupInit() {
        commands.init(hardwareMap, false);
        robot.init(hardwareMap, false);

        if (robot.hasCamera) {
            tfod = new TfodProcessor.Builder()
                    .setModelAssetName(TFOD_MODEL_ASSEST)
                    .setModelLabels(LABELS)
                    .build();

            tfod.setMinResultConfidence(.5f);
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .addProcessor(tfod)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                    .build();
        }

        //initially turn off the tagProcessor
        //visionPortal.setProcessorEnabled(tagProcessor, false);

        telemetry.addData("Status:", "ready");
        startingAngle = commands.getAngle();
        telemetry.addData("angle", startingAngle);

        telemetry.update();
    }

    protected void executeOperations(TapeColor color, CenterStageEnums.StrafeDirection parkingDirection) throws InterruptedException {
        if (robot.hasDroneSecureServo) {
            robot.droneSecureServo.setPosition(Commands.DRONE_SECURE);
        }
        if (robot.hasBlinkin) {
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
        }

        runtime.reset();
        telemetry.setAutoClear(false);
        telemetry.addData("Color: ", color);

        // Identify pixel/team element placement
        TapeLocation tapeLocation = TapeLocation.Center;

        tapeLocation = getTfodRecognitions(tfod);
        if (tapeLocation == TapeLocation.None) tapeLocation = TapeLocation.Center;

        telemetry.addData("Tape location: ", tapeLocation);
        AprilTag aprilTag = GetTagID(color, tapeLocation);
        telemetry.addData("April tag: ", aprilTag);
        telemetry.update();

        // Move off the start
        commands.driveForward(DRIVE_SPEED, 26, 3);

        // Turn AprilTag processor ON and tfod processor OFF
        // visionPortal.setProcessorEnabled(tagProcessor, true);
        visionPortal.setProcessorEnabled(tfod, false);

        // Turn to tape line
        TurnTowardsSpikeMark(color, tapeLocation);

        // Find tape line
        boolean isStrafe = (tapeLocation == TapeLocation.Left);
        commands.ApproachTape(color, isStrafe, 3);

        // Orient for pixel placement
        if (tapeLocation != TapeLocation.Left)
            commands.driveBackwards(DRIVE_SPEED_FAST, 3, 3);

        // Deliver ground pixel
        commands.deliverSpikeMarkPixel();

        if (tapeLocation == TapeLocation.Left)
            commands.driveBackwards(DRIVE_SPEED_FAST, 2, 3);

        sleep(250);
        if (tapeLocation == TapeLocation.Center) {
            commands.driveBackwards(DRIVE_SPEED_FAST, 5, 3);
        }
        if (tapeLocation == TapeLocation.Left) {
            commands.strafeRight(DRIVE_SPEED_FAST, 9, 3);
        }

        // Orient towards backdrop red/blue
        if (color == TapeColor.Red && tapeLocation == TapeLocation.Center ||
                color == TapeColor.Red && tapeLocation == TapeLocation.Left) {
            commands.spinRight(DRIVE_SPEED_FAST, -89, 5);
            commands.strafeLeft(DRIVE_SPEED_FAST, 2, 2);
        } else if (color == TapeColor.Blue) {
            if (tapeLocation == TapeLocation.Right) {
                commands.driveBackwards(DRIVE_SPEED_FAST, 5, 3);
            }
            commands.spinLeft(DRIVE_SPEED, 89, 5);
            if (tapeLocation == TapeLocation.Center) {
                commands.strafeRight(DRIVE_SPEED_FAST, 2, 2);
            }
        }
        telemetry.update();

        runtime.reset();
        boolean tagsFound = false;
        while (!isStopRequested() && robot.hasCamera && runtime.seconds() < 7 && !tagsFound) {
            tagsFound = GetCloserToAprilTags(color, 8, aprilTag);
            targetDistance -= 8;
            sleep(500);
        }

        runtime.reset();

        // Drive towards backdrop find April tag
        while (!isStopRequested() && robot.hasCamera && runtime.seconds() < 6 && targetDistance > 10) {
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Rotate, aprilTag.getValue());
            commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Strafe, aprilTag.getValue());
            double distance = commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Straight, aprilTag.getValue());
            if (distance > 0) {
                targetDistance = distance;
                tagsFound = true;
            }
        }

        //final turn towards the backdrop
        faceTowardsBackdrop(color);

        // Deliver pixel on backdrop
        if (tagsFound) {
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
        if (parkingDirection == Left) {
            commands.strafeLeft(DRIVE_SPEED_FAST, GetStrafeDistance(parkingDirection, aprilTag), 5);
        }
        if (parkingDirection == Right) {
            commands.strafeRight(DRIVE_SPEED_FAST, GetStrafeDistance(parkingDirection, aprilTag), 5);
        }

        commands.driveForward(DRIVE_SPEED_FAST, 8, 1);
        if (!tagsFound) {
            commands.driveForward(DRIVE_SPEED, targetDistance, 1);
        }
    }

    private TapeLocation getTfodRecognitions(TfodProcessor tfod) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //telemetry.addData("recognitions", currentRecognitions.size());
        int i = 0;
        // telemetry.addData("recognition", currentRecognitions);
        while (i < 10 && (currentRecognitions == null || currentRecognitions.size() == 0)) {
            currentRecognitions = tfod.getRecognitions();
            // telemetry.addData("inside recognitions", i);
            //telemetry.addData("recognitions", currentRecognitions.size());
            telemetry.update();
            sleep(200);
            i++;
        }
        if (currentRecognitions == null) return TapeLocation.None;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

//            telemetry.addData("", " ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            telemetry.update();
            //sleep(1000);
            int cameraLeft = CAMERA_WIDTH / 3;
            int cameraRight = CAMERA_WIDTH / 3 * 2;

            if (x <= cameraLeft) return TapeLocation.Left;
            if (x > cameraLeft && x < cameraRight) return TapeLocation.Center;
            if (x >= cameraRight) return TapeLocation.Right;
        }   // end for() loop

        return TapeLocation.None;
    }

    private boolean GetCloserToAprilTags(TapeColor color, int distance, AprilTag aprilTag) throws InterruptedException {
        boolean tagsFound = commands.followTag(tagProcessor, CenterStageEnums.FollowDirection.Straight, aprilTag.getValue()) > 0;
        if (!tagsFound)
            commands.driveForward(DRIVE_SPEED_FAST, distance, 3);

        faceTowardsBackdrop(color);
        return tagsFound;
    }
    private void faceTowardsBackdrop(TapeColor color){
        if (color == TapeColor.Blue) {
            if (commands.getAngle() > 90) commands.spinRight(DRIVE_SPEED, 90, 2);
            else if (commands.getAngle() < 90) commands.spinLeft(DRIVE_SPEED, 90, 2);
        }
        if (color == TapeColor.Red) {
            if (commands.getAngle() > -90) commands.spinRight(DRIVE_SPEED, -90, 2);
            else if (commands.getAngle() < -90) commands.spinLeft(DRIVE_SPEED, -90, 2);
        }
    }

    private void TurnTowardsSpikeMark(TapeColor color, TapeLocation tapeLocation) {
        if (color == TapeColor.Red) {
            if (tapeLocation == TapeLocation.Left) {
                commands.strafeLeft(DRIVE_SPEED, 6, 2);
            }
            if (tapeLocation == TapeLocation.Right) {
                commands.spinRight(DRIVE_SPEED, -90, 3);
            }
        }
        if (color == TapeColor.Blue) {
            if (tapeLocation == TapeLocation.Left) {
                commands.strafeLeft(DRIVE_SPEED, 6, 2);
            }
            if (tapeLocation == TapeLocation.Right) {
                commands.spinRight(DRIVE_SPEED, -90, 3);
            }
        }
    }

    private int GetStrafeDistance(CenterStageEnums.StrafeDirection direction, AprilTag aprilTag) {
        int distance = 27;
        if ((direction == Right && (aprilTag == AprilTag.RedRight || aprilTag == AprilTag.BlueRight)) ||
                (direction == Left && (aprilTag == AprilTag.RedLeft || aprilTag == AprilTag.BlueLeft)))
            distance = 20;

        if ((direction == Right && (aprilTag == AprilTag.RedLeft || aprilTag == AprilTag.BlueLeft)) ||
                (direction == Left && (aprilTag == AprilTag.RedRight || aprilTag == AprilTag.BlueRight)))
            distance = 33;

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