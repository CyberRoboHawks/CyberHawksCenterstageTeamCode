package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.CenterStageEnums.Position.Down;
import static org.firstinspires.ftc.teamcode.CenterStageEnums.Position.Up;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareMapping {
    //region robot "has" properties
    public boolean hasArmDownSensor = false;
    public boolean hasArmMotors = false;
    public boolean hasBlinkin = false;
    public boolean hasCamera = false;
    public boolean hasColorSensor = false;
    public boolean hasDriveMotors = false;
    public boolean hasDroneServo = false;
    public boolean hasDroneSecureServo = false;
    public boolean hasGrabberDistance = false;
    public boolean hasGrabberServo = false;

    public boolean hasLinearActuatorMotor = false;
    public boolean hasPixelServo = false;
    public boolean hasWristServo = false;
    //endregion

    public boolean isReverse = false;
    public DcMotor armMotorRight = null;
    public DcMotor armMotorLeft = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor linearActuatorMotor = null;
    public DistanceSensor grabberDistance;
    public WebcamName webCam1 = null;
    public ColorSensor colorSensor = null;
    public Servo droneServo = null;
    public Servo droneSecureServo = null;
    public Servo grabberServo = null;
  //  public CRServo gripperSlideServo = null;
    public Servo pixelServo = null;
    public Servo wristServo = null;
    public TouchSensor armDownSensor = null;
    public IMU imu = null;

    public int armPositionTarget = 0;

    public RevBlinkinLedDriver blinkinLedDriver;

    public double GRABBER_CLOSED = .8;
    public double GRABBER_OPEN = .55;

    public double WRIST_UP = .57;
    public double WRIST_DOWN = .16;
    //public double WRIST_BACKDROP = .57;

    CenterStageEnums.Position armPosition = Down;
    HardwareMap hardwareMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardware, boolean isTeleOp) {
        // Save reference to Hardware map
        hardwareMap = hardware;

        if (canGetDevice("blinkin")) {
            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
            hasBlinkin = true;
        }

        if (canGetDevice("armMotorRight"))
            armMotorRight = setupMotor("armMotorRight", DcMotor.Direction.FORWARD, 0, true, true);
        if (canGetDevice("armMotorLeft"))
            armMotorLeft = setupMotor("armMotorLeft", DcMotor.Direction.REVERSE, 0, true, true);
        hasArmMotors = (canGetDevice("armMotorRight") && canGetDevice("armMotorLeft"));
        if (hasArmMotors) {
            armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (canGetDevice("Webcam 1")) {
            webCam1 = setupWebcam("Webcam 1");
            hasCamera = true;
        }

        if (canGetDevice("colorSensor")) {
            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            hasColorSensor = true;
        }
        if (canGetDevice("armDownSensor")) {
            armDownSensor = hardwareMap.get(TouchSensor.class, "armDownSensor");
            hasArmDownSensor = true;
            if (armDownSensor.isPressed()) {
                armPosition = Down;
            } else {
                armPosition = Up;
            }
        }

        if (canGetDevice("leftFrontMotor") && canGetDevice("leftBackMotor")
                && canGetDevice("rightFrontMotor") && canGetDevice("rightBackMotor")) {
            leftFrontMotor = setupMotor("leftFrontMotor", DcMotor.Direction.REVERSE, 0, true, true);
            leftBackMotor = setupMotor("leftBackMotor", DcMotor.Direction.REVERSE, 0, true, true);
            rightFrontMotor = setupMotor("rightFrontMotor", DcMotor.Direction.FORWARD, 0, true, true);
            rightBackMotor = setupMotor("rightBackMotor", DcMotor.Direction.FORWARD, 0, true, true);
            hasDriveMotors = true;
        }

        //region Servos
        if (canGetDevice("droneServo")) {
            droneServo = setupServo("droneServo", 0);
            hasDroneServo = true;
        }
        if (canGetDevice("droneSecureServo")) {
            if (isTeleOp)
                droneSecureServo = setupServo("droneSecureServo", Commands.DRONE_SECURE);
            else
                droneSecureServo = setupServo("droneSecureServo", Commands.DRONE_FIRE);
            hasDroneSecureServo = true;
        }
        if (canGetDevice("grabberServo")) {
            grabberServo = setupServo("grabberServo", GRABBER_CLOSED);
            hasGrabberServo = true;
        }
        if (canGetDevice("pixelServo")) {
            pixelServo = setupServo("pixelServo", 0.23);
            hasPixelServo = true;
        }

        if (canGetDevice("wristServo")) {
            wristServo = setupServo("wristServo", WRIST_UP);
            hasWristServo = true;
        }

        //endregion

        if (canGetDevice("grabberDistance")) {
            grabberDistance = hardwareMap.get(DistanceSensor.class, "grabberDistance");
            hasGrabberDistance = true;
        }

        if (canGetDevice("linearActuatorMotor")) {
            linearActuatorMotor = setupMotor("linearActuatorMotor", DcMotor.Direction.FORWARD, 0, true, true);
            hasLinearActuatorMotor = true;
        }

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void stopAndRestMotorEncoders(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean canGetDevice(String deviceName) {
        try {
            hardwareMap.get(deviceName);
            return true;
        } catch (Exception ex) {
            return false;
        }
    }

    /* Init Motor, set direction, initial power and encoder runmode (if applicable)
     * @return the configured DcMotor or null if the motor is not found
     */
    private DcMotor setupMotor(String name, DcMotorSimple.Direction direction, int initialPower, boolean useEncoder, boolean brakeMode) {
        try {

            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(direction);
            motor.setPower(initialPower);

            if (useEncoder) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (brakeMode) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            return motor;
        } catch (Exception e) {
            return null;
        }
    }

    /* Init CRServo and set initial power
     * @return the configured CRServo or null if the servo is not found
     */
    private CRServo setupCRServo(String name, double initialPower) {
        try {
            CRServo servo = hardwareMap.get(CRServo.class, name);
            servo.setPower(initialPower);
            return servo;
        } catch (Exception e) {
            return null;
        }
    }

    /* Init Servo and set initial position
     * @return the configured Servo or null if the servo is not found
     */
    private Servo setupServo(String name, double initialPosition) {
        try {
            Servo servo = hardwareMap.get(Servo.class, name);
            servo.setPosition(initialPosition);
            return servo;
        } catch (Exception e) {
            return null;
        }
    }

    /* Init WebcamName
     * @return the configured WebcamName or null if the webcam is not found
     */
    private WebcamName setupWebcam(String name) {
        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, name);
            WebcamConfiguration y;

            return webcamName;
        } catch (Exception e) {
            return null;
        }
    }
}