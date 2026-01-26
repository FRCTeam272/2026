package frc.lib.utils;

import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * a wrapper for SparkMax motor controllers
 */
public class SparkMAXContainer implements MotorContainer {

  // these are public so any additional changes (current limits for example) can
  // be done in real time if needed
  public SparkMax motor;
  public RelativeEncoder encoder;
  private SparkMaxConfig config;
  public int port;

  /**
   * Creates a new SparkMAXContainer with the given id ASSUMES THE MOTOR IS
   * BRUSHLESS
   * 
   * @param id the CAN ID of the motor
   */
  public SparkMAXContainer(int id) {
    this(id, true);
  }

  /**
   * Creates a new SparkMAXContainer with the given id and brushless ness
   * 
   * @param id          CAN ID of the motor
   * @param isBrushless is the motor brushless
   */
  public SparkMAXContainer(int id, boolean isBrushless) {
    port = id;
    motor = new SparkMax(
        id,
        isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
    config = new SparkMaxConfig();
    motor.setCANTimeout(20);
    if (isBrushless) {
      encoder = motor.getEncoder();
    } else {
      encoder = null;
    }
    // we do this here becuase its like instant and we can overide all of them below
    // if needed

  }
  
  /**
   * Exposes the closed loop controller of the motor
   * you probably don't need this and if you are calling it I hope phil is sitting next to you with the docs open
   * Godspeed brave soul
   * @return the closed loop controller of the motor
   */
  public SparkClosedLoopController exposeReference(){
    return this.motor.getClosedLoopController();
  }

  /**
   * Assigns the defualt PID values to the motor assumes P = 0.1, I = 0, D = 0
   */
  @Override
  public void assignPIDValues() {
    assignPIDValues(0.1, 0, 0);
  }

  /**
   * Assigns the PID values to the motor
   * 
   * @param P the P value
   * @param I the I value
   * @param D the D value
   */
  @Override
  public void assignPIDValues(double P, double I, double D) {
    config.closedLoop.p(P).i(I).d(D);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
  * @param kS UNITS: Volts
  * @param kV UNITS:Volts per velocity, DESC: Volts per motor RPM by default
  * @param kA UNITS:Volts per velocity/s, DESC: Volts per motor RPM/s by default
  * @param kG UNITS:Volts, DESC Elevator/linear mechanism gravity feedforward
  * @param kCos UNITS:Volts, DESC Arm/rotary mechanism gravity feedforward. Feedback sensor must be configured to 0 = horizontal
  * @paramCosRatio UNITS: Ratio, Converts feedback sensor readings to mechanism rotations
  * @see https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
  */ 
  public void assignFF(double kS, double kV, double kA, double kG, double kCos, double kCosRatio){
    config.closedLoop.feedForward.kA(kA).kV(kV).kA(kA).kG(kG).kCos(kCos).kCosRatio(kCosRatio);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void assignFF(double kS, double kV, double kA, double kG){
    config.closedLoop.feedForward.kA(kA).kV(kV).kA(kA).kG(kG);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Assigns this motor to follow another SparkMaxContainer
   * 
   * @param leader the spark max this should follow
   * @param invert weither or not this motor should be inverted from the other
   *               motor
   */
  public void setupAsFollowerMotor(MotorContainer leader, boolean invert) throws IllegalArgumentException {
    if(leader instanceof SparkMAXContainer) {
      SparkMAXContainer lead = (SparkMAXContainer) leader;
      config.follow(lead.port, invert);
      motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    } else {
      DriverStation.reportError("SparkMAXContainer can only follow other SparkMAXContainers", false);
      throw new IllegalArgumentException("SparkMAXContainer can only follow other SparkMAXContainers");
    }
  }

  /**
   * Sets the gear ratio of the motor for finner postion control
   * 
   * @param gearRatio can be represeneted via a fraction
   */
  public void setGearRatio(double gearRatio) {
    // setPositionConversionFactor
    config.encoder.positionConversionFactor(gearRatio);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Sends the motor to a specific position, returns true if it is within the
   * deadband (4 encoder ticks)
   * 
   * @param pos desired postion
   * @return true when within deadband
   */
  public boolean goToPostion(double pos) {
    return this.goToPostion(pos, 4);
  }

  /**
   * Sends the motor to a specific position, returns true if it is within the
   * deadband
   * 
   * @param pos      desired postion
   * @param deadband the deadband to be within, deadband should not be 0 but can
   *                 be as small as 1
   * @return true when within deadband
   */
  public boolean goToPostion(double pos, int deadband) {
    try {
      var encoderPos = encoder.getPosition();
      motor.getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
      return encoderPos > pos - deadband || encoderPos < encoderPos + deadband;
    } catch (Exception e) {
      DriverStation.reportError(e.getMessage(), false);
      return true;
    }
  }

  /**
   * Sets the current limit of the motor
   * primarily uses smart current limit but has the secondary current limit as +5
   * as a saftey
   * @param limit
   */
  public void setCurrentLimit(double limit) {
    this.setSecondaryCurrentLimit(100);
    this.setSmartCurrentLimit((int)limit);
  }

  public void setInverted(boolean value){
    this.config.inverted(value);
    this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Sets the smart current limit of the motor
   * @param limit
   */
  public void setSmartCurrentLimit(int limit){
    config.smartCurrentLimit(limit);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Sets the secondary current limit of the motor
   * @param limit
   */
  public void setSecondaryCurrentLimit(double limit) {
    config.secondaryCurrentLimit(limit);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Sets the break mode of the motor
   * 
   * @param isBreakMode true for break mode, false for coast mode
   */
  public void setBreakMode(boolean isBreakMode) {
    config.idleMode(isBreakMode ? IdleMode.kBrake : IdleMode.kCoast);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getMotorTemperatureInC() {
    return this.motor.getMotorTemperature();
  }

  public double getMotorTemperatureInF() {
    return (this.getMotorTemperatureInC() * 9 / 5) + 32;
  }

  /**
   * sets the max and min output for the motor by a fraction
   * @param speed postive number < 1
   */
  public void setMaxSpeed(double speed){
    speed = Math.abs(speed);
    this.config.closedLoop.outputRange(-speed, speed);
    this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getPosition(){
    if(encoder == null){
      DriverStation.reportError("Trying to get position of a brushed motor without an encoder", false);
      return 0;
    }
    return encoder.getPosition();
  }

  /**
   * gets the velocity of the motor in RPM
   * @return velocity in RPM
   */
  public double getVelocity(){
    if(encoder == null){
      DriverStation.reportError("Trying to get velocity of a brushed motor without an encoder", false);
      return 0;
    }
    return encoder.getVelocity();
  }

  /**
   * sets the velocity of the motor in RPM
   * @param velocity desired velocity in RPM
   * @return the set velocity
   */
  public double setVelocity(double velocity){
    motor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
    return this.getVelocity();
  }
  
  /**
   * Sends information about the motor to SmartDashboard, these calls should be
   * contained in an if(!DriverStation.isFMSAttached()) block to avoid flooding
   * the system in competition
   * 
   * @param key the head name of the motor
   */
  private boolean canReadTemp = true;
  public void reportMotor(String key) {
    SmartDashboard.putNumber(key + "/Encoder Value", encoder.getPosition());
    SmartDashboard.putNumber(key + "/Velocity", encoder.getVelocity());
    SmartDashboard.putNumber(key + "/Current", motor.getOutputCurrent());
    SmartDashboard.putNumber(key + "/Applied Output", motor.getAppliedOutput());
    SmartDashboard.putNumber(key + "/CurrentLimit/Smart Limit", motor.configAccessor.getSmartCurrentLimit());
    SmartDashboard.putNumber(key + "/CurrentLimit/Secondary Limit", motor.configAccessor.getSecondaryCurrentLimit()); 
    try{
      if (canReadTemp) {
        SmartDashboard.putNumber(key + "/Motor Temp (F)", this.getMotorTemperatureInF());
        SmartDashboard.putNumber("Temps(F)/" + key, this.getMotorTemperatureInF());
        SmartDashboard.putNumber("Temps(C)/" + key, this.getMotorTemperatureInC());
      }
    } catch (Exception e){
      canReadTemp = false;
    }
    
  }
}
