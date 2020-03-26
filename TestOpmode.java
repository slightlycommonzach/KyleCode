package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.math.BigDecimal;

@TeleOp(name = "KyleDrive")
public class TestOpmode extends LinearOpMode {
    private DcMotor lMotor;
    private DcMotor rMotor;
    private DcMotor backMotor;
    private DcMotor frontMotor;
    private BNO055IMU imu;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    private DcMotor lArmMotor;
    private DcMotor rArmMotor;
    private DcMotor elbowMotor;
    private Servo gripServo;

    public void runOpMode() {
        final BigDecimal pi = new BigDecimal(Math.PI);
        final double power = 1;
        lMotor = hardwareMap.get(DcMotor.class, "lMotor");
        rMotor = hardwareMap.get(DcMotor.class, "rMotor");
        frontMotor = hardwareMap.get(DcMotor.class, "frontMotor");
        backMotor = hardwareMap.get(DcMotor.class, "backMotor");
        lArmMotor = hardwareMap.get(DcMotor.class, "lArmMotor");
        rArmMotor = hardwareMap.get(DcMotor.class, "rArmMotor");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbowMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        double cal = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) + (Math.PI / 2);

        waitForStart();
// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            if (gamepad1.right_bumper){
//
//            }
            telemetry.addData("Status", "Running");
            telemetry.update();
            //   float theta = Math.sqrt(pi.multiply(imu.getAngularVelocityAxes());
            double gyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - cal;
            double theta = (gyroAngle - Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y));
//            double mag = Math.sqrt(Math.pow(gamepad1.left_stick_x,2)+ Math.pow(gamepad1.left_stick_y,2));
            double mag = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("Angle", gyroAngle);
            telemetry.addData("Theta", theta);
            lMotor.setPower(Range.clip(mag * Math.sin(theta) - gamepad1.right_stick_x, -1, 1) * power);
            rMotor.setPower(Range.clip(mag * Math.sin(theta) + gamepad1.right_stick_x, -1, 1) * power);
            frontMotor.setPower(Range.clip(mag * Math.cos(theta) + gamepad1.right_stick_x, -1, 1) * power);
            backMotor.setPower(Range.clip(mag * Math.cos(theta) - gamepad1.right_stick_x, -1, 1) * power);
// To make it not field oriented use this
            //            lMotor.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) * power);
//            rMotor.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.right_stick_x, -1, 1) * power);
//            frontMotor.setPower(Range.clip(gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1) * power);
//            backMotor.setPower(Range.clip(gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1) * power);
            telemetry.addData("Pos: ", lMotor.getCurrentPosition());


        }
    }

    //    @Override
//    public void loop() {
//        telemetry.addData("Pos: ", lMotor.getCurrentPosition());
//        lMotor.setPower(1);
//        telemetry.update();
//
//    }
   // fps MAX
  //  RGB MAX
   // ACTIVATE KILLMODE
   // DOWNLOAD ALL
   // RAM
}
