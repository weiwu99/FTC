/**
 * Created by pliu on 1/16/18.
 */

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.Locale;

/**
 * Created by dwu on 1/15/18.
 */

@Autonomous(name = "coreAuto", group = "5100")
@Disabled
public abstract class coreAuto extends LinearOpMode {

    private DcMotor LeftFront, RightFront, LeftBack, RightBack;
    private DcMotor Lifting;

    private Servo leftservo, rightservo;
    private Servo bottom, top;

    private ColorSensor sensorColor;

    private double initHeading;

    private BNO055IMU imu;

    private Orientation angles;


    public void runSuperMeta(int position, int key) {

        boolean Red = getSide(position);

        hardwareInitialize();

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        composeTelemetry();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //Start
        if(opModeIsActive()){

            colorR(Red);

            //Motion
            initHeading = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
            composeTelemetry();
            telemetry.update();
            runMeta(position, key);
        }
    }

    void hardwareInitialize(){

        LeftFront = hardwareMap.dcMotor.get("LF");
        RightFront = hardwareMap.dcMotor.get("RF");
        LeftBack = hardwareMap.dcMotor.get("LB");
        RightBack = hardwareMap.dcMotor.get("RB");

        Lifting = hardwareMap.dcMotor.get("LS");

        leftservo = hardwareMap.servo.get("servo1");
        rightservo = hardwareMap.servo.get("servo2");

        bottom = hardwareMap.servo.get("servo3");
        top = hardwareMap.servo.get("servo4");

    }

    boolean getSide(int index){
        if(index == 1 || index == 2) return true;
        return false;
    }

    boolean colorDetect(){
        sensorColor = hardwareMap.get(ColorSensor.class, "sensorcd");

        boolean isRed = false;

        float hsvValues[] = {0F, 0F, 0F};

        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;

        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        sleep(1000);

        while(opModeIsActive()) {

            double red = sensorColor.red();
            double blue = sensorColor.blue();

            if ( red < blue || Math.abs(red - blue) <= 5) { isRed = false; break;}

            else if (red - blue > 5) { isRed = true; break;}
        }

        return isRed;
    }

    void colorR(boolean red){


        leftservo.setPosition(1);
        rightservo.setPosition(0);
        sleep(300);

        top.setPosition(0.95);
        sleep(300);

        boolean isRed = colorDetect();

        if (red == true) {
            if (isRed == true) {
                bottom.setPosition(0.6);
                sleep(300);
                bottom.setPosition(0.75);
                sleep(200);

            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.85);
                sleep(200);
            }
        } else {
            if (isRed == false) {
                bottom.setPosition(0.6);
                sleep(300);
                bottom.setPosition(0.75);
                sleep(200);

            } else {
                bottom.setPosition(1);
                sleep(300);
                bottom.setPosition(0.85);
                sleep(200);
            }
        }
        top.setPosition(0);
        sleep(500);

    }

    void turnCheck(double degree){

        double temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
        double sub = temp;

        if(degree > 0) {
            while (Math.abs(sub - initHeading - degree) > 10) {
                LeftFront.setPower(-0.5);
                LeftBack.setPower(-0.5);
                RightFront.setPower(-0.5);
                RightBack.setPower(-0.5);
                sleep(200);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp < 0)
                    sub = 360 + temp;
            }
        }
        else if (degree < 0) {
            while (Math.abs(sub - initHeading - degree) > 10) {
                LeftFront.setPower(0.5);
                LeftBack.setPower(0.5);
                RightFront.setPower(0.5);
                RightBack.setPower(0.5);
                sleep(200);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp > 0)
                    sub = temp - 360;
            }
        }

        initHeading += degree;

        if(sub - degree > 0) {
            while (Math.abs(sub - degree) > 2 && degree < sub) {
                LeftFront.setPower(0.3);
                LeftBack.setPower(0.3);
                RightFront.setPower(0.3);
                RightBack.setPower(0.3);
                sleep(100);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;

                if(temp > 0 && degree < 0)
                    sub = temp - 360;
                if(temp < 0 && degree > 0)
                    sub = 360 + temp;
            }
        }
        else if (sub - degree < 0) {
            while (Math.abs(sub - degree) > 2 && degree > sub) {
                LeftFront.setPower(-0.3);
                LeftBack.setPower(-0.3);
                RightFront.setPower(-0.3);
                RightBack.setPower(-0.3);
                sleep(100);
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
                composeTelemetry();
                telemetry.update();
                sleep(100);
                temp = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
                sub = temp;
                if(temp > 0 && degree < 0)
                    sub = temp - 360;
                if(temp < 0 && degree > 0)
                    sub = 360 + temp;
            }
        }
    }

    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void movement(double pwr, int time, String direction) {
        switch (direction) {
            case "forward":
                LeftFront.setPower(pwr);
                LeftBack.setPower(pwr);
                RightFront.setPower(-pwr);
                RightBack.setPower(-pwr);
                break;
            case "backward":
                LeftFront.setPower(-pwr);
                LeftBack.setPower(-pwr);
                RightFront.setPower(pwr);
                RightBack.setPower(pwr);
                break;
            case "left":
                LeftFront.setPower(-pwr);
                LeftBack.setPower(pwr);
                RightFront.setPower(-pwr);
                RightBack.setPower(pwr);
                break;
            case "right":
                LeftFront.setPower(pwr);
                LeftBack.setPower(-pwr);
                RightFront.setPower(pwr);
                RightBack.setPower(-pwr);
                break;
        }

        sleep(time);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

    }

    public void runMeta (int posi, int key) {


        Lifting.setPower(-0.5);
        sleep(1000);
        Lifting.setPower(0);

        switch (posi) {
            case 1:
                movement(0.2, 1700, "backward");

                if(key == 0) {
                    turnCheck(-116);
                    movement(0.4, 200, "left");
                    movement(0.3, 300, "forward");
                }
                else if(key == 1) {
                    turnCheck(-126.5);
                }
                else if(key == 2) {
                    turnCheck(-161);
                }
                else {
                    turnCheck(-126.5);
                }

                movement(0.3, 350, "forward");
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                movement(0.3, 550, "forward");

                leftservo.setPosition(0.5);
                rightservo.setPosition(0.5);
                sleep(500);
                movement(0.3, 350, "forward");
                movement(0.3, 200, "backward");
                if(key == 0) {
                    movement(0.3, 100, "backward");
                }
                break;

            case 2:

                if(key == 0) {
                movement(0.2,3200, "backward");
                turnCheck(68);
                movement(0.4,300,"left");
            }

            else if(key == 1) {
                movement(0.2, 2600, "backward");
                turnCheck(70);
            }

            else if(key == 2) {
                movement(0.2,2200, "backward");
                turnCheck(68);
                movement(0.4,300,"right");
            }
            else {
                movement(0.2, 2600, "backward");
                turnCheck(70);
            }


            movement(0.3, 200, "forward");
            Lifting.setPower(0.5);
            sleep(800);
            Lifting.setPower(0);
            sleep(200);

            movement(0.3, 500, "forward");

            leftservo.setPosition(0.5);
            rightservo.setPosition(0.5);
            sleep(500);
            movement(0.3,600, "forward");
            movement(0.3,200,"backward");
            break;

            case 3:

                if(key == 0) {
                    movement(0.2,2200, "forward");
                    turnCheck(112);
                    movement(0.4,300,"left");
                }

                else if(key == 1) {
                    movement(0.2, 2600, "forward");
                    turnCheck(110);
                }

                else if(key == 2) {
                    movement(0.2,3200, "forward");
                    turnCheck(112);
                    movement(0.4,300,"right");
                }
                else {
                    movement(0.2, 2600, "forward");
                    turnCheck(70);
                }

                movement(0.3, 200, "forward");
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                movement(0.3, 500, "forward");

                leftservo.setPosition(0.5);
                rightservo.setPosition(0.5);
                sleep(500);
                movement(0.3,600, "forward");
                movement(0.3,200, "backward");
                break;

            case 4:
                movement(0.4, 700, "forward");

                if(key == 0) {
                    turnCheck(-19);
                }
                else if(key == 1) {
                    turnCheck(-53.5);
                }
                else if(key == 2) {
                    turnCheck(-61);
                    movement(0.4, 200, "right");
                    movement(0.3, 300, "forward");
                }
                else {
                    turnCheck(-53.5);
                }

                movement(0.3, 300, "forward");
                Lifting.setPower(0.5);
                sleep(800);
                Lifting.setPower(0);
                sleep(200);

                movement(0.3, 500, "forward");

                leftservo.setPosition(0);
                rightservo.setPosition(1);
                sleep(500);
                movement(0.3, 300, "forward");
                movement(0.3, 200, "backward");
                if(key == 2) {
                    movement(0.3, 200, "backward");
                }
                break;
        }
    }

}