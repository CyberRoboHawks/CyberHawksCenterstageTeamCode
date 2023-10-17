package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.WebcamConfiguration;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareMapping {
    //region robot "has" properties
    public boolean hasArmMotors = false;
    public boolean hasCamera = false;
    public boolean hasDriveMotors = false;
    public boolean hasDroneServo = false;
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
    public WebcamName webCam1 = null;
    public TouchSensor limitSwitchIn = null;
    public TouchSensor limitSwitchOut = null;
    public Servo droneServo = null;
    public Servo grabberServo = null;
    public Servo pixelServo = null;
    public CRServo wristServo = null;

    public IMU imu = null;
    public ElapsedTime runtime = new ElapsedTime();
    HardwareMap hardwareMap = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardware) {
        // Save reference to Hardware map
        hardwareMap = hardware;

        if (canGetDevice("armMotorRight"))
            armMotorRight = setupMotor("armMotorRight", DcMotor.Direction.FORWARD, 0, true, true);
        if (canGetDevice("armMotorLeft"))
            armMotorLeft = setupMotor("armMotorLeft", DcMotor.Direction.FORWARD, 0, true, true);
        hasArmMotors = (canGetDevice("armMotorRight") && canGetDevice("armMotorLeft"));

        if (canGetDevice("Webcam 1")) {
            webCam1 = setupWebcam("Webcam 1");
            hasCamera = true;
        }

        if (canGetDevice("leftFrontMotor") && canGetDevice("leftBackMotor")
                && canGetDevice("rightFrontMotor") && canGetDevice("rightBackMotor")) {
            leftFrontMotor = setupMotor("leftFrontMotor", DcMotor.Direction.REVERSE, 0, true, true);
            leftBackMotor = setupMotor("leftBackMotor", DcMotor.Direction.REVERSE, 0, true, true);
            rightFrontMotor = setupMotor("rightFrontMotor", DcMotor.Direction.FORWARD, 0, true, true);
            rightBackMotor = setupMotor("rightBackMotor", DcMotor.Direction.FORWARD, 0, true, true);
            hasDriveMotors = true;
        }

        // Servos
        if (canGetDevice("droneServo")) {
            droneServo = setupServo("droneServo", 0);
            hasDroneServo = true;
        }
        if (canGetDevice("grabberServo")) {
            grabberServo = setupServo("grabberServo", .4);
            hasGrabberServo = true;
        }
        if (canGetDevice("pixelServo")) {
            pixelServo = setupServo("pixelServo", 0.2);
           // pixelServo.setDirection(DcMotorSimple.Direction.FORWARD);
            hasPixelServo = true;
        }
        if (canGetDevice("wristServo")) {
            wristServo = setupCRServo("wristServo", 0.00);
            wristServo.setDirection(DcMotorSimple.Direction.FORWARD);
            hasWristServo = true;
        }

        if (canGetDevice("linearActuatorMotor")) {
            linearActuatorMotor = setupMotor("linearActuatorMotor", DcMotor.Direction.FORWARD, 0, false, true);
            hasLinearActuatorMotor = true;
        }

        if (canGetDevice("limitSwitchIn"))
            limitSwitchIn = hardwareMap.get(TouchSensor.class, "limitSwitchIn");
        if (canGetDevice("limitSwitchOut"))
            limitSwitchOut = hardwareMap.get(TouchSensor.class, "limitSwitchOut");

        imu = hardwareMap.get(IMU.class, "imu");

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
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