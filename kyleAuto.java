package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@TeleOp(name = "KyleAuto")
public class kyleAuto extends OpMode {

    private final double ARM1_LENGTH = 0;
    private final double ARM2_LENGTH = 0;
    private final double BASETOELBOWDIFHYPO = 438.94375;//mm
    private final double BASETOELBOWDIFHEIGHT = 71.4375;//mm
    private final double MIDTOELBOWDIFHYPO = 400;//mm
    private final double MIDTOELBOWDIFHEIGHT = 254;//mm
    private DcMotor lMotor;
    private DcMotor rMotor;
    private DcMotor backMotor;
    private DcMotor frontMotor;
    private BNO055IMU imu;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private DcMotorEx lArmMotor;
    private DcMotorEx rArmMotor;
    private DcMotorEx elbowMotor;
    private Servo wristServo;
    private Servo gripServo;
   private PID pidArm = new PID(1, 0, 0);
    private OpenCvCamera phoneCam;
    int state = -1;
    private double armTickCal;

    @Override
    public void init() {
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");
        frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        backMotor = hardwareMap.get(DcMotor.class, "backMotor");
        lArmMotor = hardwareMap.get(DcMotorEx.class, "lArmMotor");
        rArmMotor = hardwareMap.get(DcMotorEx.class, "rArmMotor");
        elbowMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        gripServo = hardwareMap.get(Servo.class, "Gripservo");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        lArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        // sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        // servoTest = hardwareMap.get(Servo.class, "servoTest");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        lMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        double cal = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) + (Math.PI / 2);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();


        phoneCam.setPipeline(new kyleDetect());
        armTickCal = lArmMotor.getCurrentPosition();
//        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        pidArm.setTarget(90);//setting target to the top of the angle
//TODO: re-add this^
    }
// Cases:
    //case 0: firstExtenstion
// case 1: scan
// case 2: navigate
// case 3: grab
// case 4: contract
// default: tele/driver mode


    @Override
    public void loop() {
//get the positioning values from the pipeline
        double baseTick = lArmMotor.getCurrentPosition()-armTickCal;
        double baseAngleOffset = 0;//Math.asin(BASETOELBOWDIFHEIGHT / BASETOELBOWDIFHYPO);
        double baseAngle = -
                (baseTick / (753.2*3) * 360) - baseAngleOffset;//baseTick/ticksPerRev*360 - base axle to mid axle dif at rest in mm
        switch (state) {
            case 0:
                pidArm.setTarget(180);
                // rArmMotor.setPower(pidArm.pidUpdate(total amount of ticks/ current number of ticks * 360));
                // double bigArmTheta= Math.acos(Math.pow(67.0/4,2))+Math.pow(129/8,2)-Math.pow(531/16,2)/(2*(67/4)*(129/8));

                //  double topAngle = 0;
                double heightBaseAngle = Math.sin(baseAngle);//Math.sin(baseAngle) +
                double midAngleOffset = Math.asin(MIDTOELBOWDIFHEIGHT / MIDTOELBOWDIFHYPO);
                double midTick = elbowMotor.getCurrentPosition();
                double midAngle = (midTick / 753.2 * 360) - midAngleOffset;
                //    double topAngle = 0;
                double heightMidAngle = (67 / 4) * Math.sin(midAngle) + 11.5; //64/4 is the hypotenuse, i did hypo*sin(theta) = opposite (height)
                double totalHeight = heightBaseAngle + heightMidAngle;
                if (totalHeight < 30) {
                    lArmMotor.setPower(.2);
                    rArmMotor.setPower(.2);
                    elbowMotor.setPower(.2);
                } else {
                    lArmMotor.setPower(0);
                    rArmMotor.setPower(0);
                    elbowMotor.setPower(0);
                }
                telemetry.addData("height", totalHeight);
                telemetry.update();
                break;
            case 1:
                //processFrame()
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            default:

                double assistPower = -(90-baseAngle)*0.00125;
                double thetaInterior = (90-baseAngle);
                telemetry.addData("angle base",baseAngle);
                telemetry.addData("angle int",thetaInterior);
                double testAngle = ((1)*Math.sin(Math.toRadians(baseAngle)))/Math.sin(Math.toRadians(thetaInterior));
                telemetry.addData("Angle", baseAngle);//16
                telemetry.addData("Assist Power", assistPower);//4
//                lArmMotor.setPower((gamepad1.right_stick_y )+assistPower);
//                rArmMotor.setPower((gamepad1.right_stick_y )+assistPower);
                telemetry.addData("angle",testAngle);
               lArmMotor.setPower(Range.clip(pidArm.pidUpdate(-baseAngle),-0.5,0.5));
                rArmMotor.setPower(Range.clip(pidArm.pidUpdate(-baseAngle),-0.5,0.5));
                break;

        }
        telemetry.addData("state", state);
    }
}

