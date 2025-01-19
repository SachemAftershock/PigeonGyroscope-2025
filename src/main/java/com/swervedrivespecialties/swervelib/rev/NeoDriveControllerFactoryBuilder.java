package com.swervedrivespecialties.swervelib.rev;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        
        @Override
        public ControllerImplementation create(Integer id, ModuleConfiguration moduleConfiguration) {

            SparkFlex motor = new SparkFlex(id, SparkLowLevel.MotorType.kBrushless);
            SparkFlexConfig config = new SparkFlexConfig();

            // PeriodicStatus0
            config.signals.appliedOutputPeriodMs(100);
            config.signals.faultsPeriodMs(100);
            
            // PeriodicStatus1
            config.signals.absoluteEncoderVelocityPeriodMs(20);
            config.signals.busVoltagePeriodMs(20);
            config.signals.motorTemperaturePeriodMs(20);
            config.signals.outputCurrentPeriodMs(20);
            
            // PeriodicStatus2
            config.signals.primaryEncoderPositionPeriodMs(20);
            config.signals.absoluteEncoderPositionPeriodMs(20);
            config.signals.iAccumulationPeriodMs(20);

            // Additional configurations
            config.voltageCompensation(nominalVoltage);
            config.smartCurrentLimit((int)currentLimit);
            config.inverted(moduleConfiguration.isDriveInverted());
            config.idleMode(IdleMode.kBrake);

            // Setup encoder
            EncoderConfig encoderConfig = new EncoderConfig();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            encoderConfig.positionConversionFactor(positionConversionFactor);
            encoderConfig.velocityConversionFactor(positionConversionFactor / 60.0);
            config.apply(encoderConfig);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            RelativeEncoder encoder = motor.getEncoder();
          
            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final SparkFlex motor;
        private final RelativeEncoder encoder;

        private ControllerImplementation(SparkFlex motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getPosition() {
            return encoder.getPosition();
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public void setCanStatusFramePeriodReductions() {
        }
    }
}
