package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;


//Java imports


//Vendor imports
import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;

import com.studica.frc.Servo;
import com.studica.frc.ServoContinuous;

public class Training extends SubsystemBase
{
    /**
     * Motors
     */
    private TitanQuad motorL;
    private TitanQuad motorB;
    private TitanQuad motorR;
    // private Servo servo;
    private Servo servoC;
    private Servo servoU;

    private DigitalOutput ledGreen;
    private DigitalOutput ledRed;

    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private TitanQuadEncoder backEncoder;

    /**
     * Sensors
     */
    // private Cobra cobra;
    // private Ultrasonic sonic;
    // private AnalogInput sharp;
    private AHRS gyro;

    public Training()
    {
        //Motors
        motorR = new TitanQuad(Constants.TITAN_ID, Constants.MOTOR_R);
        motorB = new TitanQuad(Constants.TITAN_ID, Constants.MOTOR_B);
        motorL = new TitanQuad(Constants.TITAN_ID, Constants.MOTOR_L);
        // servo = new Servo(Constants.SERVO);
        servoU = new Servo(Constants.SERVO_U);
        servoC = new Servo(Constants.SERVO_C);

        gyro = new AHRS(SPI.Port.kMXP);
        ledGreen = new DigitalOutput(20);
        ledRed = new DigitalOutput(21);
    }

    /**
     * Call for the raw ADC value
     * <p>
     * @param channel range 0 - 3 (matches what is on the adc)
     * @return value between 0 and 2047 (11-bit)
     */


    /**
     * Call for the voltage from the ADC
     * <p>
     * @param channel range 0 - 3 (matches what is on the adc)
     * @return voltage between 0 - 5V (0 - 3.3V if the constructor Cobra(3.3F) is used)
     */

    /**
     * Call for the distance measured by the Sharp IR Sensor
     * <p>
     * @return value between 0 - 100 (valid data range is 10cm - 80cm)
     */


    /**
     * Call for the distance measured by the Ultrasonic Sensor
     * <p>
     * @param metric true or false for metric output
     * @return distance in mm when metric is true, and inches when metric is false
     */


    /**
     * Call for the current angle from the internal NavX
     * <p>
     * @return yaw angle in degrees range -180° to 180°
     */
    public double getYaw()
    {
        return gyro.getYaw();
    }

    /**
     * Resets the yaw angle back to zero
     */
    public void resetGyro()
    {
        gyro.zeroYaw();
    }

    public void holonomicDrive(double x, double y, double z)
    {
        double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double leftSpeed = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
        double backSpeed = (-2 * x / 3) + z;

        double max = Math.abs(rightSpeed);
        if (Math.abs(leftSpeed) > max) max = Math.abs(leftSpeed);
        if (Math.abs(backSpeed) > max) max = Math.abs(backSpeed);

        if (max > 1)
        {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }

        motorL.set(leftSpeed);
        motorR.set(rightSpeed);
        motorB.set(backSpeed);
    }


    /**
     * Sets the servo angle based on the input from the shuffleboard widget 
     */


    /**
     * Sets the servo angle 
     * <p>
     * @param degrees degree to set the servo to, range 0° - 300°
     */
    // public void setServoCAngle(double degrees)
    // {
    //     servoC.setAngle(degrees);
    // }

    public void setAngleServo(double degrees)
    {
        servoC.setAngle(degrees);
    }

    public void ledSetGreen(Boolean state){
        ledGreen.set(state);
    }

    public void ledSetRed(Boolean state){
        ledRed.set(state);
    }

    /**
     * Sets the servo speed based on the input from the shuffleboard widget
     */


    /**
     * Sets the servo speed
     * <p>
     * @param speed sets the speed of the servo in continous mode, range -1 to 1
     */
    public void setServoUSpeed(double speed)
    {
        servoU.set(speed);
    }

    public void setServoUStop()
    {
        servoU.stopMotor();
        
    }

    /**
     * Sets the speed of the motor
     * <p>
     * @param speed range -1 to 1 (0 stop)
     */
    public void setMotorLSpeed(double speed)
    {
        motorL.set(speed);
    }

    public void setMotorBSpeed(double speed)
    {
        motorB.set(speed);
    }

    public void setMotorRSpeed(double speed)
    {
        motorR.set(speed);
    }

    /**
     * Code that runs once every robot loop
     */
    @Override
    public void periodic()
    {

    }

	public void setDriveMotorSpeeds(double d, double e, double f) {
    }
    
    //DriveWithPID
    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }

    public void resetYaw()
    {
        gyro.zeroYaw();
    }

    public double getLeftEncoderDistance()
    {
        return leftEncoder.getEncoderDistance();
    }

    public double getRightEncoderDistance()
    {
        return rightEncoder.getEncoderDistance() * -1;
    }

    public double getAverageForwardEncoderDistance()
    {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2; 
    }
}