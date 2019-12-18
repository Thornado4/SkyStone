package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="MecanumDrive", group="OpMode")
public class MecanumDrive extends OpMode {

    DcMotor motorDriveLF;
    DcMotor motorDriveLB;
    DcMotor motorDriveRF;
    DcMotor motorDriveRB;
    BNO055IMU imu;                  // IMU Gyro itself
    Orientation angles;

    double angleTest[] = new double[10];
    int count = 0;
    double sum;
    double correct;
    double liftPowerShift = 1;

    @Override
    public void init() {
        motorDriveLF = hardwareMap.dcMotor.get("motorDriveLF");
        motorDriveLB = hardwareMap.dcMotor.get("motorDriveLB");
        motorDriveRF = hardwareMap.dcMotor.get("motorDriveRF");
        motorDriveRB = hardwareMap.dcMotor.get("motorDriveRB");

        motorDriveLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDriveRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI/4);
            angle += angles.firstAngle;
            double turnPower = gamepad1.right_stick_x;

            if(turnPower == 0){
                if (count < 10) {
                    angleTest[count] = angle;
                    angleTest[count] = angle;
                    count++;
                }
                else if (count >= 10){
                    sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0] + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8] + angleTest[9])/10;
                    if(sum > angle){
                        correct = sum - angle;
                        angle = angle + correct;
                    }
                    else if(angle > sum){
                        correct = angle - sum;
                        angle = angle - correct;
                    }
                    count = 0;

                }
            }

            motorDriveLF.setPower(((speed * (Math.sin(angle)) + turnPower)));
            motorDriveRF.setPower(((speed * -(Math.cos(angle))) + turnPower));
            motorDriveLB.setPower(((speed * (Math.cos(angle)) + turnPower)));
            motorDriveRB.setPower(((speed * -(Math.sin(angle))) + turnPower));
        }
    }