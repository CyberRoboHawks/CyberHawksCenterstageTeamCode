package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CenterStageEnums.StrafeDirection.Left;
import static org.firstinspires.ftc.teamcode.CenterStageEnums.StrafeDirection.Right;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.CenterStageEnums.AprilTag;
import org.firstinspires.ftc.teamcode.CenterStageEnums.TapeColor;
import org.firstinspires.ftc.teamcode.CenterStageEnums.TapeLocation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public abstract class AutonomousBaseTfod extends LinearOpMode {
    static final float DRIVE_SPEED = 0.3f;
    static final float DRIVE_SPEED_FAST = 0.6f;

    static final float ROTATE_SPEED = 0.3f;

    private final ElapsedTime runtime = new ElapsedTime();

    //    private static final String TFOD_MODEL_ASSEST = "CyberHawks_1.tflite";
//    private static  final String[] LABELS = {
//        "Blue Prop",
//            "Red Prop"
//    };
    Commands commands = new Commands(telemetry);
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    AprilTagProcessor tagProcessor = null;
    VisionPortal visionPortal = null;
    double startingAngle;
    private TfodProcessor tfod;
     double totalSpikeDistance = 0;
     double goalSpikeDistance = 26;

    public void startupInit() {
        commands.init(hardwareMap);
        robot.init(hardwareMap);

        if (robot.hasCamera) {
//            tfod = new TfodProcessor.Builder()
//                    .setModelAssetName(TFOD_MODEL_ASSEST)
//                    .setModelLabels(LABELS)
//                    .build();

            tfod = TfodProcessor.easyCreateWithDefaults();
            tfod.setMinResultConfidence(.6f);
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .build();
            visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .addProcessor(tfod)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    //.setCameraResolution(new Size(800, 600))
                  //  .setCameraResolution(new Size(1920, 1080))
                    .build();
        }

        telemetry.addData("Status:", "ready");
        startingAngle = commands.getAngle();
        telemetry.addData("angle", startingAngle);
        //   getTfodRecognitions(tfod);
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

        TapeLocation tapeLocation = getTfodRecognitions(tfod);
        // Move off the start
        commands.driveForward(DRIVE_SPEED, 8, 3);
        totalSpikeDistance += 8;

        tapeLocation = getTfodRecognitions(tfod);
        // Identify pixel/team element placement

        if (tapeLocation == TapeLocation.None){
            commands.spinLeft(ROTATE_SPEED, 20, 2);
            sleep(500);
            tapeLocation = getTfodRecognitions(tfod);
            telemetry.addData("after", "first check");
            telemetry.update();
            if (tapeLocation == TapeLocation.None) {
                tapeLocation = getTfodRecognitions(tfod);
            }
            commands.spinRight(ROTATE_SPEED, 0, 2);
        }

        if (tapeLocation == TapeLocation.None ){
            int count = 0;
            while (count < 3 && tapeLocation == TapeLocation.None) {
                telemetry.addData("count", count);
                telemetry.update();
                tapeLocation = getTfodRecognitions(tfod);
                if (tapeLocation != TapeLocation.None) break;
                commands.driveForward(DRIVE_SPEED, 4, 2);
                totalSpikeDistance += 4;
                sleep(1000);
                tapeLocation = getTfodRecognitions(tfod);

                if (tapeLocation != TapeLocation.None) break;
                count++;
            }
        }
        commands.driveForward(DRIVE_SPEED, goalSpikeDistance - totalSpikeDistance, 2);

        telemetry.addData("Tape location: ", tapeLocation);
        if (tapeLocation == TapeLocation.None) tapeLocation = TapeLocation.Center;

        telemetry.addData("Tape location: ", tapeLocation);
        AprilTag aprilTag = GetTagID(color, tapeLocation);
        telemetry.addData("April tag: ", aprilTag);
        telemetry.update();

        // commands.driveForward(DRIVE_SPEED, 26, 3);

        // tapeLocation = getTfodRecognitions(tfod);

        // Turn to tape line
        TurnTowardsSpikeMark(color, tapeLocation);

        // Find tape line
        boolean isStrafe = (tapeLocation == TapeLocation.Left);
        commands.ApproachTape(color, isStrafe, 3);

        sleep(30000);

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
        if (robot.isRoboHawks) {
            if (color == TapeColor.Red)
                commands.spinLeft(ROTATE_SPEED, 90, 6);
            else
                commands.spinRight(ROTATE_SPEED, -90, 6);

            commands.reverseDriveMotorDirection();
            // sleep(250);
            //Raise arm
            commands.setArmPositionRH(CenterStageEnums.ArmDirection.Up);
            commands.moveLinearActuatorToPosition(commands.LINEAR_FLOOR, commands.LINEAR_POWER);

            sleep(250);
            commands.approachBackdrop(6, 4);

            // open grabber
            robot.gripperSlideServo.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.gripperSlideServo.setPower(.5);
            sleep(250);
            robot.gripperSlideServo.setPower(0);

            // reverse the linear actuator
            commands.moveLinearActuatorToPosition(commands.LINEAR_MIN, commands.LINEAR_POWER);
            sleep(1300);

            commands.setArmPositionRH(CenterStageEnums.ArmDirection.Down);
            sleep(250);
            commands.driveBackwards(DRIVE_SPEED_FAST, 4, 2);
            commands.setArmPower(0);
        }

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

    private TapeLocation getTfodRecognitions(TfodProcessor tfod) {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("recognitions", currentRecognitions.size());
        int i = 0;
        telemetry.addData("recognition", currentRecognitions);
        while (i < 10 && (currentRecognitions == null || currentRecognitions.size() ==0)){
            currentRecognitions = tfod.getFreshRecognitions();
            telemetry.addData("inside recognitions", i);
            //telemetry.addData("recognitions", currentRecognitions.size());
            telemetry.update();
            sleep(100);
            i++;
        }
        //if (currentRecognitions == null) return TapeLocation.None;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
            sleep(1000);
            if (x <= 200) return TapeLocation.Left;
            if (x > 200 && x < 400) return TapeLocation.Center;
            if (x >= 400) return TapeLocation.Right;


//            if (x <= 650) return TapeLocation.Left;
//            if (x > 650 && x < 1300) return TapeLocation.Center;
//            if (x >= 1300) return TapeLocation.Right;
        }   // end for() loop

        return TapeLocation.None;
    }

    private boolean GetCloserToAprilTags(TapeColor color, int distance) {
        ArrayList<AprilTagDetection> detections = tagProcessor.getFreshDetections();
        boolean tagsFound = detections.size() > 0;
        telemetry.addData("detections: ", detections.size());
        telemetry.update();
        if (!tagsFound)
            commands.driveForward(DRIVE_SPEED_FAST, distance, 3);

        if (color == TapeColor.Blue) {
            if (commands.getAngle() > 90) commands.spinRight(ROTATE_SPEED, 90, 2);
            else if (commands.getAngle() < 90) commands.spinLeft(ROTATE_SPEED, 90, 2);
        }
        if (color == TapeColor.Red) {
            if (commands.getAngle() > -90) commands.spinRight(ROTATE_SPEED, -90, 2);
            else if (commands.getAngle() < -90) commands.spinLeft(ROTATE_SPEED, -90, 2);
        }
        return tagsFound;
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
        int distance = 30;
        if ((direction == Right && (aprilTag == AprilTag.RedRight || aprilTag == AprilTag.BlueRight)) ||
                (direction == Left && (aprilTag == AprilTag.RedLeft || aprilTag == AprilTag.BlueLeft)))
            distance = 20;

        if ((direction == Right && (aprilTag == AprilTag.RedLeft || aprilTag == AprilTag.BlueLeft)) ||
                (direction == Left && (aprilTag == AprilTag.RedRight || aprilTag == AprilTag.BlueRight)))
            distance = 30;

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