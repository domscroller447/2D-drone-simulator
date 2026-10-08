#include "Drone.h"

#include <cmath>

#include "raylib.h"

namespace {

// Degrees-to-radians conversion factor. Defined locally (rather than relying
// on the non-standard M_PI macro) to keep the file portable across
// toolchains without requiring _USE_MATH_DEFINES on MSVC.
constexpr float kDegToRadian = 3.14159265358979323846f / 180.0f;

// Sprite and HUD layout. These are presentation details of Draw() only, not
// part of the drone's physical/control model, so they live here rather than
// in Drone.h.
constexpr float kSpriteWidthPx = 80.0f;
constexpr float kSpriteHeightPx = 20.0f;
constexpr int kHudMarginPx = 10;
constexpr int kHudLineHeightPx = 25;
constexpr int kHudFontSize = 20;

// Telemetry log destination and column header. Implementation detail of
// HandleInput()'s logging toggle, not class state.
constexpr const char* kLogFilePath = "C:/Users/ADMIN/Desktop/drone_sim/flight_log.csv";
constexpr const char* kLogHeader =
    "Time,X,TrueHeight,RawHeight,FilteredHeight,Vx,RawVx,FilteredVx,Vy,Pitch,Thrust\n";

}  // namespace

Drone::Drone(float startX, float startY)
    : x_(startX / kPixelsPerMeter), y_(startY / kPixelsPerMeter) {}

void Drone::HandleInput(float dt) {
    bool manualSteering = false;

    if (!positionHoldEnabled_) {
        if (IsKeyDown(KEY_RIGHT)) {
            pitch_ += kPitchRateDegPerSec * dt;
            manualSteering = true;
        }
        if (IsKeyDown(KEY_LEFT)) {
            pitch_ -= kPitchRateDegPerSec * dt;
            manualSteering = true;
        }
    }

    if (IsKeyPressed(KEY_A)) {
        altitudeHoldEnabled_ = !altitudeHoldEnabled_;
        integralErrorAltitude_ = 0.0f;
        if (altitudeHoldEnabled_) {
            captureAltitudeNextFrame_ = true;
        }
    }

    if (IsKeyPressed(KEY_P)) {
        positionHoldEnabled_ = !positionHoldEnabled_;
        integralErrorX_ = 0.0f;
        if (positionHoldEnabled_) {
            altitudeHoldEnabled_ = true;
            captureAltitudeNextFrame_ = true;
            capturePositionNextFrame_ = true;
        }
    }

    if (IsKeyPressed(KEY_W)) windEnabled_ = !windEnabled_;
    if (IsKeyPressed(KEY_N)) noiseEnabled_ = !noiseEnabled_;
    if (IsKeyPressed(KEY_F)) filterEnabled_ = !filterEnabled_;

    if (IsKeyPressed(KEY_L)) {
        loggingEnabled_ = !loggingEnabled_;
        if (loggingEnabled_) {
            logFile_.open(kLogFilePath);
            if (logFile_.is_open()) {
                logFile_ << kLogHeader;
            }
        } else if (logFile_.is_open()) {
            logFile_.close();
        }
    }

    if (!positionHoldEnabled_ && !manualSteering) {
        if (pitch_ > kPitchAutoLevelDeadbandDeg) {
            pitch_ -= kPitchRateDegPerSec * dt;
        } else if (pitch_ < -kPitchAutoLevelDeadbandDeg) {
            pitch_ += kPitchRateDegPerSec * dt;
        } else {
            pitch_ = 0.0f;
        }
    }

    if (altitudeHoldEnabled_) {
        if (IsKeyDown(KEY_UP)) {
            targetHeight_ += kAltitudeTargetRateMps * dt;
        }
        if (IsKeyDown(KEY_DOWN)) {
            targetHeight_ -= kAltitudeTargetRateMps * dt;
        }
        if (targetHeight_ < kMinTargetAltitudeM) {
            targetHeight_ = kMinTargetAltitudeM;
        }
        if (targetHeight_ > kMaxTargetAltitudeM) {
            targetHeight_ = kMaxTargetAltitudeM;
        }
    }
}

void Drone::UpdatePhysics(float dt, int screenWidth, int screenHeight) {
    const float screenBottom = static_cast<float>(screenHeight - kWallMarginPx) / kPixelsPerMeter;
    const float trueHeight = screenBottom - y_;

    // 1. Raw measurement generation with Gaussian IMU noise.
    if (noiseEnabled_) {
        rawHeightMeasurement_ = trueHeight + heightNoiseDist_(rng_);
        rawVelocityXMeasurement_ = vx_ + velocityNoiseDist_(rng_);
    } else {
        rawHeightMeasurement_ = trueHeight;
        rawVelocityXMeasurement_ = vx_;
    }

    // Seed the filter with the first valid measurement so it doesn't
    // produce a transient spike while converging from an initial value
    // of zero.
    if (filteredHeight_ == 0.0f && trueHeight > 0.0f) {
        filteredHeight_ = trueHeight;
        filteredVelocityX_ = vx_;
    }

    // 2. Exponential moving average low-pass filtering.
    if (filterEnabled_) {
        filteredHeight_ = (kFilterAlpha * rawHeightMeasurement_) + ((1.0f - kFilterAlpha) * filteredHeight_);
        filteredVelocityX_ =
            (kFilterAlpha * rawVelocityXMeasurement_) + ((1.0f - kFilterAlpha) * filteredVelocityX_);
    } else {
        filteredHeight_ = rawHeightMeasurement_;
        filteredVelocityX_ = rawVelocityXMeasurement_;
    }

    const float feedbackHeight = filteredHeight_;
    const float feedbackVelocityX = filteredVelocityX_;

    if (captureAltitudeNextFrame_) {
        targetHeight_ = feedbackHeight;
        captureAltitudeNextFrame_ = false;
    }
    if (capturePositionNextFrame_) {
        targetX_ = x_;
        capturePositionNextFrame_ = false;
    }

    float appliedThrust = 0.0f;

    // 3. Altitude-hold PID loop.
    if (altitudeHoldEnabled_) {
        const float errorAltitude = targetHeight_ - feedbackHeight;
        integralErrorAltitude_ += errorAltitude * dt;
        if (integralErrorAltitude_ > kMaxAltitudeIntegralError) {
            integralErrorAltitude_ = kMaxAltitudeIntegralError;
        }
        if (integralErrorAltitude_ < -kMaxAltitudeIntegralError) {
            integralErrorAltitude_ = -kMaxAltitudeIntegralError;
        }

        const float pidOutputAltitude =
            (kAltitudeHoldKp * errorAltitude) + (kAltitudeHoldKi * integralErrorAltitude_) + (kAltitudeHoldKd * vy_);
        appliedThrust = (kMass * kGravity) + pidOutputAltitude;

        if (appliedThrust < 0.0f) {
            appliedThrust = 0.0f;
        }
        if (appliedThrust > kMaxThrust) {
            appliedThrust = kMaxThrust;
        }
    } else if (IsKeyDown(KEY_SPACE)) {
        appliedThrust = kMaxThrust;
    }

    // 4. Position-hold PID loop.
    if (positionHoldEnabled_) {
        const float errorX = targetX_ - x_;
        integralErrorX_ += errorX * dt;

        float desiredTilt = (kPositionHoldKp * errorX) - (kPositionHoldKd * feedbackVelocityX);
        if (desiredTilt > kMaxPositionHoldTiltDeg) {
            desiredTilt = kMaxPositionHoldTiltDeg;
        }
        if (desiredTilt < -kMaxPositionHoldTiltDeg) {
            desiredTilt = -kMaxPositionHoldTiltDeg;
        }

        pitch_ = desiredTilt;
    }

    // 5. Physics integration.
    float forceX = 0.0f;
    float forceY = kMass * kGravity;

    if (windEnabled_) {
        const float time = static_cast<float>(GetTime());
        const float windForce = kWindBaseForceN + kWindGust1AmplitudeN * sinf(time * kWindGust1AngularFreq) +
                                 kWindGust2AmplitudeN * cosf(time * kWindGust2AngularFreq);
        forceX += windForce;
    }

    const float pitchRadians = pitch_ * kDegToRadian;
    forceX += appliedThrust * sinf(pitchRadians);
    forceY -= appliedThrust * cosf(pitchRadians);

    const float speed = sqrtf(vx_ * vx_ + vy_ * vy_);
    if (speed > 0.0f) {
        const float dragForce = 0.5f * kAirDensity * speed * speed * kFrontalArea * kDragCoefficient;
        forceX -= dragForce * (vx_ / speed);
        forceY -= dragForce * (vy_ / speed);
    }

    vx_ += (forceX / kMass) * dt;
    vy_ += (forceY / kMass) * dt;
    x_ += vx_ * dt;
    y_ += vy_ * dt;

    if (loggingEnabled_ && logFile_.is_open()) {
        logFile_ << GetTime() << ',' << x_ << ',' << trueHeight << ',' << rawHeightMeasurement_ << ','
                  << filteredHeight_ << ',' << vx_ << ',' << rawVelocityXMeasurement_ << ',' << filteredVelocityX_
                  << ',' << vy_ << ',' << pitch_ << ',' << appliedThrust << '\n';
    }

    const float screenRight = static_cast<float>(screenWidth - kWallMarginPx) / kPixelsPerMeter;
    if (y_ > screenBottom) {
        y_ = screenBottom;
        vy_ = 0.0f;
        vx_ *= kGroundFrictionFactor;
    }
    if (y_ < kMinYPositionM) {
        y_ = kMinYPositionM;
        vy_ = 0.0f;
    }
    if (x_ < kMinXPositionM) {
        x_ = kMinXPositionM;
        vx_ = 0.0f;
    }
    if (x_ > screenRight) {
        x_ = screenRight;
        vx_ = 0.0f;
    }
}

void Drone::Draw() const {
    const Rectangle droneRect = {x_ * kPixelsPerMeter, y_ * kPixelsPerMeter, kSpriteWidthPx, kSpriteHeightPx};
    const Vector2 origin = {kSpriteWidthPx / 2.0f, kSpriteHeightPx / 2.0f};
    DrawRectanglePro(droneRect, origin, pitch_, DARKGRAY);

    DrawText(TextFormat("Airspeed: %.1f m/s", sqrtf(vx_ * vx_ + vy_ * vy_)), kHudMarginPx, kHudMarginPx,
             kHudFontSize, DARKBLUE);

    const char* modeText = positionHoldEnabled_     ? "POS HOLD: ON (P)"
                            : altitudeHoldEnabled_   ? "ALT HOLD: ON (A)"
                                                      : "MANUAL FLIGHT";
    const Color modeColor = positionHoldEnabled_    ? DARKPURPLE
                             : altitudeHoldEnabled_  ? DARKGREEN
                                                      : RED;
    DrawText(modeText, kHudMarginPx, kHudMarginPx + kHudLineHeightPx, kHudFontSize, modeColor);

    DrawText(windEnabled_ ? "WIND: ACTIVE (W)" : "WIND: OFF (W)", kHudMarginPx,
             kHudMarginPx + 2 * kHudLineHeightPx, kHudFontSize, windEnabled_ ? ORANGE : DARKGRAY);
    DrawText(noiseEnabled_ ? "IMU NOISE: ON (N)" : "IMU NOISE: OFF (N)", kHudMarginPx,
             kHudMarginPx + 3 * kHudLineHeightPx, kHudFontSize, noiseEnabled_ ? RED : DARKGRAY);
    DrawText(filterEnabled_ ? "EMA FILTER: ON (F)" : "EMA FILTER: OFF (F)", kHudMarginPx,
             kHudMarginPx + 4 * kHudLineHeightPx, kHudFontSize, filterEnabled_ ? GREEN : DARKGRAY);
    DrawText(loggingEnabled_ ? "LOGGING: RECORDING (L)" : "LOGGING: IDLE (L)", kHudMarginPx,
             kHudMarginPx + 5 * kHudLineHeightPx, kHudFontSize, loggingEnabled_ ? MAROON : DARKGRAY);

    if (altitudeHoldEnabled_) {
        DrawText(TextFormat("Target Alt: %.1f m | Est Alt: %.1f m", targetHeight_, filteredHeight_), kHudMarginPx,
                 kHudMarginPx + 6 * kHudLineHeightPx, kHudFontSize, DARKBLUE);
    }
}