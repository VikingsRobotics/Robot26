#pragma once
#ifndef CONSTANTS_PID_CONSTANTS_H
#define CONSTANTS_PID_CONSTANTS_H

#include <units/base.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <pathplanner/lib/config/PIDConstants.h>

namespace Gains {

/**
 * @brief Dimensionally-safe PID gain container.
 *
 * @tparam Output The controller output quantity type
 *         (volt_t, ampere_t, scalar_t, etc).
 *
 * @tparam Error The controlled process variable type
 *         (turn_t, meter_t, etc).
 *
 * @tparam Time Time unit of the mechanism
 *         (seconds, minutes, milliseconds, etc)
 *
 * This struct enforces dimensional correctness at compile time using
 * WPILib's units library. Gains are defined such that:
 *
 *   output =
 *       kP * error
 *     + kI * ∫error dt
 *     + kD * d(error)/dt
 */
template <typename Output, typename Error, typename Time>
struct PIDConstants {
    using kP_t = units::unit_t<units::compound_unit<typename Output::unit_type, units::inverse<typename Error::unit_type>>>;

    using kI_t = units::unit_t<
        units::compound_unit<typename Output::unit_type, units::inverse<units::compound_unit<typename Error::unit_type, typename Time::unit_type>>>>;

    using kD_t = units::unit_t<units::compound_unit<typename Output::unit_type,
                                                    units::inverse<units::compound_unit<typename Error::unit_type, units::inverse<typename Time::unit_type>>>>>;

    /**
     * @brief Proportional gain.
     *
     * Units: Output / Error
     */
    kP_t kP{};

    /**
     * @brief Integral gain.
     *
     * Units: Output / (Error * second)
     */
    kI_t kI{};

    /**
     * @brief Derivative gain.
     *
     * Units: Output / (Error / second)
     */
    kD_t kD{};

    PIDConstants() = default;
    PIDConstants(kP_t kP, kI_t kI, kD_t kD) { WithPID(kP, kI, kD); }

    PIDConstants& WithP(kP_t value) {
        kP = value;
        return *this;
    }

    PIDConstants& WithI(kI_t value) {
        kI = value;
        return *this;
    }

    PIDConstants& WithD(kD_t value) {
        kD = value;
        return *this;
    }

    PIDConstants& WithPID(kP_t p, kI_t i, kD_t d) {
        kP = p;
        kI = i;
        kD = d;
        return *this;
    }
};


/**
 * @brief Dimensionally-safe feedforward gain container.
 *
 * @tparam Output Controller output quantity
 *         (volt_t, ampere_t, scalar_t, etc).
 *
 * @tparam Velocity Velocity of the mechanism
 *         (turns_per_second_t, meters_per_second_t, rpm, etc).
 *
 * @tparam Time Time unit of the mechanism
 *         (seconds, minutes, milliseconds, etc).
 *
 * Feedforward model:
 *
 *   output =
 *       kS * sign(velocity)
 *     + kV * velocity
 *     + kA * acceleration
 */
template <typename Output, typename Velocity, typename Time>
struct FFConstants {
    using kS_t = units::unit_t<typename Output::unit_type>;

    using kV_t = units::unit_t<units::compound_unit<typename Output::unit_type, units::inverse<typename Velocity::unit_type>>>;

    using kA_t =
        units::unit_t<units::compound_unit<typename Output::unit_type,
                                           units::inverse<units::compound_unit<typename Velocity::unit_type, units::inverse<typename Time::unit_type>>>>>;
    /**
     * @brief Static gain (overcomes static friction).
     *
     * Units: Output
     */
    kS_t kS{};

    /**
     * @brief Velocity gain.
     *
     * Units: Output / Velocity
     */
    kV_t kV{};

    /**
     * @brief Acceleration gain.
     *
     * Units: Output / (Velocity / second)
     */
    kA_t kA{};

    FFConstants() = default;
    FFConstants(kS_t kS, kV_t kV, kA_t kA) { WithSVA(kS, kV, kA); }

    FFConstants& WithS(kS_t value) {
        kS = value;
        return *this;
    }

    FFConstants& WithV(kV_t value) {
        kV = value;
        return *this;
    }

    FFConstants& WithA(kA_t value) {
        kA = value;
        return *this;
    }

    FFConstants& WithSVA(kS_t s, kV_t v, kA_t a) {
        kS = s;
        kV = v;
        kA = a;
        return *this;
    }
};


/**
 * @brief Combined PID + Feedforward gains.
 *
 * @tparam Output Controller output type.
 * @tparam Error Position or process variable type.
 * @tparam Velocity Velocity type of the mechanism.
 * @tparam Time Time type of the mechanism
 */
template <typename Output, typename Error, typename Velocity, typename Time>
struct GainConstants {
    using PID = PIDConstants<Output, Error, Time>;
    using FF = FFConstants<Output, Velocity, Time>;

    using kP_t = typename PID::kP_t;
    using kI_t = typename PID::kI_t;
    using kD_t = typename PID::kD_t;
    using kS_t = typename FF::kS_t;
    using kV_t = typename FF::kV_t;
    using kA_t = typename FF::kA_t;

    PID pid{};  /// Feedback gains
    FF sva{};   /// Feedforward gains

    GainConstants() = default;
    GainConstants(PID pid, FF ff) {
        WithPID(pid);
        WithSVA(ff);
    }

    GainConstants& WithP(kP_t p) {
        pid.kP = p;
        return *this;
    }

    GainConstants& WithI(kI_t i) {
        pid.kI = i;
        return *this;
    }

    GainConstants& WithD(kD_t d) {
        pid.kD = d;
        return *this;
    }

    GainConstants& WithPID(kP_t p, kI_t i, kD_t d) {
        pid.WithPID(p, i, d);
        return *this;
    }

    GainConstants& WithPID(const PID& feedback) {
        pid = feedback;
        return *this;
    }

    GainConstants& WithS(kS_t s) {
        sva.kS = s;
        return *this;
    }

    GainConstants& WithV(kV_t v) {
        sva.kV = v;
        return *this;
    }

    GainConstants& WithA(kA_t a) {
        sva.kA = a;
        return *this;
    }

    GainConstants& WithSVA(kS_t s, kV_t v, kA_t a) {
        sva.WithSVA(s, v, a);
        return *this;
    }

    GainConstants& WithSVA(const FF& feedforward) {
        sva = feedforward;
        return *this;
    }
};

//
// -------------------------
// Rotational Mechanisms (RPM)
// -------------------------
//

using DutyCycleGainRevolution = GainConstants<units::dimensionless::scalar_t, units::turn_t, units::revolutions_per_minute_t, units::second_t>;

using VoltageGainRevolution = GainConstants<units::volt_t, units::turn_t, units::revolutions_per_minute_t, units::second_t>;

using TorqueGainRevolution = GainConstants<units::ampere_t, units::turn_t, units::revolutions_per_minute_t, units::second_t>;


//
// -------------------------
// Linear Distance Mechanisms
// -------------------------
//

using DutyCycleGainDistance = GainConstants<units::dimensionless::scalar_t, units::meter_t, units::meters_per_second_t, units::second_t>;

using VoltageGainDistance = GainConstants<units::volt_t, units::meter_t, units::meters_per_second_t, units::second_t>;

using TorqueGainDistance = GainConstants<units::ampere_t, units::meter_t, units::meters_per_second_t, units::second_t>;


//
// -------------------------
// PID-Only Aliases (Rotation)
// -------------------------
//

using DutyCyclePIDRotation = PIDConstants<units::dimensionless::scalar_t, units::turn_t, units::second_t>;

using VoltagePIDRotation = PIDConstants<units::volt_t, units::turn_t, units::second_t>;

using TorquePIDRotation = PIDConstants<units::ampere_t, units::turn_t, units::second_t>;

//
// -------------------------
// Feedforward-Only Aliases (Rotation)
// -------------------------
//

using DutyCycleFFRevolution = FFConstants<units::dimensionless::scalar_t, units::revolutions_per_minute_t, units::second_t>;

using VoltageFFRevolution = FFConstants<units::volt_t, units::revolutions_per_minute_t, units::second_t>;

using TorqueFFRevolution = FFConstants<units::ampere_t, units::revolutions_per_minute_t, units::second_t>;


//
// -------------------------
// PID-Only Aliases (Distance)
// -------------------------
//

using DutyCyclePIDDistance = PIDConstants<units::dimensionless::scalar_t, units::meter_t, units::second_t>;

using VoltagePIDDistance = PIDConstants<units::volt_t, units::meter_t, units::second_t>;

using TorquePIDDistance = PIDConstants<units::ampere_t, units::meter_t, units::second_t>;


//
// -------------------------
// Feedforward-Only Aliases (Distance)
// -------------------------
//

using DutyCycleFFDistance = FFConstants<units::dimensionless::scalar_t, units::meters_per_second_t, units::second_t>;

using VoltageFFDistance = FFConstants<units::volt_t, units::meters_per_second_t, units::second_t>;

using TorqueFFDistance = FFConstants<units::ampere_t, units::meters_per_second_t, units::second_t>;


//
// -------------------------
// Feedforward-Only Aliases (Rotation)
// Rev had a weird one
// -------------------------
//

using DutyCyclePIDRotationMs = Gains::PIDConstants<units::dimensionless::scalar_t, units::turn_t, units::millisecond_t>;


template <typename Output, typename Error, typename Time>
pathplanner::PIDConstants ConvertPIDToPathplanner(PIDConstants<Output, Error, Time> pidConst, double iZone) {
    return pathplanner::PIDConstants{pidConst.kP(), pidConst.kI(), pidConst.kD(), iZone};
}

template <typename Output, typename Error, typename Time>
PIDConstants<Output, Error, Time> ConvertPathplannerToPID(pathplanner::PIDConstants pathPID) {
    return PIDConstants<Output, Error, Time>{}
        .WithP(typename PIDConstants<Output, Error, Time>::kP_t{pathPID.kP})
        .WithI(typename PIDConstants<Output, Error, Time>::kI_t{pathPID.kI})
        .WithD(typename PIDConstants<Output, Error, Time>::kD_t{pathPID.kD});
}

}  // namespace Gains


#endif