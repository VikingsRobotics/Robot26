#include "swerve/PIDController.hpp"

#include <frc/MathUtil.h>

#include <algorithm>
#include <cmath>

PIDController::PIDController(double Kp, double Ki, double Kd) : m_Kp{Kp}, m_Ki{Ki}, m_Kd{Kd} {
    if (Kp < 0.0 || Ki < 0.0 || Kd < 0.0) {
        m_Kp = 0.0;
        m_Ki = 0.0;
        m_Kd = 0.0;
    }
}

void PIDController::SetPID(double Kp, double Ki, double Kd) {
    SetP(Kp);
    SetI(Ki);
    SetD(Kd);
}

void PIDController::SetP(double Kp) {
    m_Kp = Kp > 0.0 ? Kp : 0.0;
}

void PIDController::SetI(double Ki) {
    m_Ki = Ki > 0.0 ? Ki : 0.0;
}

void PIDController::SetD(double Kd) {
    m_Kd = Kd > 0.0 ? Kd : 0.0;
}

void PIDController::SetIZone(double iZone) {
    m_iZone = iZone > 0.0 ? iZone : 0.0;
}

double PIDController::GetP() const {
    return m_Kp;
}

double PIDController::GetI() const {
    return m_Ki;
}

double PIDController::GetD() const {
    return m_Kd;
}

double PIDController::GetIZone() const {
    return m_iZone;
}

double PIDController::GetPositionTolerance() const {
    return m_positionTolerance;
}

double PIDController::GetVelocityTolerance() const {
    return m_velocityTolerance;
}

double PIDController::GetSetpoint() const {
    return m_setpoint;
}

bool PIDController::AtSetpoint() const {
    if (m_haveSetpoint && m_haveMeasurement) {
        return (std::abs(m_positionError) < m_positionTolerance && std::abs(m_velocityError) < m_velocityTolerance);
    }
    return false;
}

void PIDController::EnableContinuousInput(double minimumInput, double maximumInput) {
    if (maximumInput > minimumInput) {
        m_minimumInput = minimumInput;
        m_maximumInput = maximumInput;
        m_continuous = true;
    }
}

void PIDController::DisableContinuousInput() {
    m_continuous = false;
}

bool PIDController::IsContinuousInputEnabled() const {
    return m_continuous;
}

void PIDController::SetIntegratorRange(double minimumIntegral, double maximumIntegral) {
    if (maximumIntegral > minimumIntegral) {
        m_minimumIntegral = minimumIntegral;
        m_maximumIntegral = maximumIntegral;
    }
}

void PIDController::SetTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance > 0 ? positionTolerance : 0.05;
    m_velocityTolerance = velocityTolerance > 0 ? velocityTolerance : std::numeric_limits<double>::infinity();
}

double PIDController::GetPositionError() const {
    return m_positionError;
}

double PIDController::GetVelocityError() const {
    return m_velocityError;
}

double PIDController::Calculate(double measurement, units::second_t currentTimestamp) {
    return Calculate(measurement, m_setpoint, currentTimestamp);
}

double PIDController::Calculate(double measurement, double setpoint, units::second_t currentTimestamp) {
    double timeDiff = currentTimestamp.value() - m_lastTimestamp.value();
    timeDiff = timeDiff <= 0 ? 1e-6 : timeDiff;

    m_lastTimestamp = currentTimestamp;

    m_measurement = measurement;
    m_setpoint = setpoint;
    m_haveSetpoint = true;

    double error = m_setpoint - m_measurement;

    if (m_continuous) {
        double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
        error = frc::InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
    }

    m_prevError = m_positionError;
    m_positionError = error;

    if (m_haveMeasurement) {
        m_velocityError = (m_positionError - m_prevError) / timeDiff;

        if (std::abs(m_positionError) <= m_iZone) {
            if (m_Ki != 0.0) {
                m_totalError += m_positionError * timeDiff;

                double minIntegral = m_minimumIntegral / m_Ki;
                double maxIntegral = m_maximumIntegral / m_Ki;

                m_totalError = std::clamp(m_totalError, minIntegral, maxIntegral);
            }
        } else {
            m_totalError = 0.0;
        }
    } else {
        m_velocityError = 0.0;
        m_totalError = 0.0;
    }

    m_haveMeasurement = true;

    return m_lastAppliedOutput = m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
}

double PIDController::GetLastAppliedOutput() const {
    return m_lastAppliedOutput;
}

void PIDController::Reset() {
    m_positionError = 0.0;
    m_velocityError = 0.0;
    m_prevError = 0.0;
    m_totalError = 0.0;
    m_haveMeasurement = false;
}