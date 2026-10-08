#pragma once

#include <fstream>
#include <random>

/// A 2D drone with Newtonian rigid-body dynamics, dual-loop PID
/// stabilization (altitude hold + position hold), a simulated IMU noise
/// model, and an exponential-moving-average (EMA) state filter. Flight
/// telemetry can be logged to CSV for offline analysis.
///
/// Unit convention: positions/distances in meters, velocities in m/s,
/// angles in degrees, time in seconds, forces in newtons. Pixel values
/// only appear at the render/physics boundary and are converted via
/// kPixelsPerMeter.
class Drone {
public:
    /// @param startX Initial horizontal position, in screen pixels.
    /// @param startY Initial vertical position, in screen pixels.
    Drone(float startX, float startY);

    // Drone owns a file handle (logFile_). RAII members (std::ofstream,
    // std::default_random_engine) already manage their own resources
    // correctly, so no custom destructor is declared (Rule of Zero).
    // Copying is explicitly disabled to document that a Drone's identity
    // (and its open log file) is not meant to be duplicated.
    Drone(const Drone&) = delete;
    Drone& operator=(const Drone&) = delete;

    /// Reads keyboard input for this frame and updates control-mode
    /// flags, setpoints, and manual pitch/thrust commands.
    /// @param dt Frame delta time, in seconds.
    void HandleInput(float dt);

    /// Advances physics integration, the sensor noise/filter models, and
    /// both PID control loops by one timestep.
    /// @param dt Frame delta time, in seconds.
    /// @param screenWidth Current window width, in pixels.
    /// @param screenHeight Current window height, in pixels.
    void UpdatePhysics(float dt, int screenWidth, int screenHeight);

    /// Renders the drone sprite and the on-screen telemetry HUD.
    void Draw() const;

private:
    // -------------------------------------------------------------------
    // Physical & aerodynamic properties
    // -------------------------------------------------------------------
    static constexpr float kMass = 1.5f;                       ///< kg
    static constexpr float kGravity = 9.81f;                   ///< m/s^2
    static constexpr float kMaxThrust = 30.0f;                 ///< N
    static constexpr float kPitchRateDegPerSec = 100.0f;       ///< manual pitch rate
    static constexpr float kPitchAutoLevelDeadbandDeg = 0.5f;  ///< snaps to 0 within this band

    static constexpr float kAirDensity = 1.225f;   ///< kg/m^3
    static constexpr float kDragCoefficient = 1.2f;
    static constexpr float kFrontalArea = 0.05f;   ///< m^2

    static constexpr float kPixelsPerMeter = 100.0f;

    // -------------------------------------------------------------------
    // World boundaries (used by physics for ground/wall clamping)
    // -------------------------------------------------------------------
    static constexpr int kWallMarginPx = 20;              ///< inset from window edges
    static constexpr float kMinYPositionM = 0.1f;         ///< keeps drone below the top edge
    static constexpr float kMinXPositionM = 0.2f;         ///< keeps drone right of the left edge
    static constexpr float kGroundFrictionFactor = 0.85f; ///< horizontal speed retained on landing

    // -------------------------------------------------------------------
    // Altitude-hold PID loop
    // -------------------------------------------------------------------
    static constexpr float kAltitudeHoldKp = 10.0f;
    static constexpr float kAltitudeHoldKi = 0.5f;
    static constexpr float kAltitudeHoldKd = 4.0f;
    static constexpr float kMaxAltitudeIntegralError = 3.0f;  ///< anti-windup clamp
    static constexpr float kMinTargetAltitudeM = 0.5f;
    static constexpr float kMaxTargetAltitudeM = 5.5f;
    static constexpr float kAltitudeTargetRateMps = 1.0f;     ///< manual climb/descend rate

    // -------------------------------------------------------------------
    // Position-hold PID loop
    // -------------------------------------------------------------------
    static constexpr float kPositionHoldKp = 2.0f;
    static constexpr float kPositionHoldKd = 1.5f;
    static constexpr float kMaxPositionHoldTiltDeg = 25.0f;

    // -------------------------------------------------------------------
    // Wind gust model: constant base force plus two out-of-phase
    // sinusoidal components.
    // -------------------------------------------------------------------
    static constexpr float kWindBaseForceN = 4.0f;
    static constexpr float kWindGust1AmplitudeN = 3.0f;
    static constexpr float kWindGust1AngularFreq = 1.5f;  ///< rad/s
    static constexpr float kWindGust2AmplitudeN = 1.5f;
    static constexpr float kWindGust2AngularFreq = 3.7f;  ///< rad/s

    // -------------------------------------------------------------------
    // Sensor noise & EMA state filtering
    // -------------------------------------------------------------------
    static constexpr float kHeightNoiseStdDev = 0.15f;    ///< m
    static constexpr float kVelocityNoiseStdDev = 0.30f;  ///< m/s
    static constexpr float kFilterAlpha = 0.15f;          ///< EMA smoothing factor

    // -------------------------------------------------------------------
    // Kinematic state
    // -------------------------------------------------------------------
    float x_ = 0.0f;      ///< m
    float y_ = 0.0f;      ///< m (screen-space convention: increases downward)
    float vx_ = 0.0f;     ///< m/s
    float vy_ = 0.0f;     ///< m/s
    float pitch_ = 0.0f;  ///< degrees

    // -------------------------------------------------------------------
    // Control-mode flags & setpoints.
    //
    // NOTE: these two flags are intentionally independent rather than a
    // single exclusive "mode" enum. Enabling position hold forces altitude
    // hold on, but disabling altitude hold afterwards (while position hold
    // stays on) is a distinct, reachable state: pitch is still auto-leveled
    // toward targetX_ while thrust reverts to manual (spacebar). Collapsing
    // this into a 3-way enum would silently remove that combination -- see
    // chat for details.
    // -------------------------------------------------------------------
    bool altitudeHoldEnabled_ = false;
    bool positionHoldEnabled_ = false;
    bool captureAltitudeNextFrame_ = false;
    bool capturePositionNextFrame_ = false;

    float targetHeight_ = 2.0f;            ///< m
    float integralErrorAltitude_ = 0.0f;

    float targetX_ = 0.0f;                 ///< m
    float integralErrorX_ = 0.0f;

    // -------------------------------------------------------------------
    // Environmental / diagnostic toggles (independent, non-exclusive)
    // -------------------------------------------------------------------
    bool windEnabled_ = false;
    bool noiseEnabled_ = false;
    bool filterEnabled_ = false;
    bool loggingEnabled_ = false;

    // -------------------------------------------------------------------
    // Sensor noise model
    // -------------------------------------------------------------------
    std::default_random_engine rng_;
    std::normal_distribution<float> heightNoiseDist_{0.0f, kHeightNoiseStdDev};
    std::normal_distribution<float> velocityNoiseDist_{0.0f, kVelocityNoiseStdDev};

    float rawHeightMeasurement_ = 0.0f;
    float rawVelocityXMeasurement_ = 0.0f;
    float filteredHeight_ = 0.0f;
    float filteredVelocityX_ = 0.0f;

    // -------------------------------------------------------------------
    // Telemetry log (RAII: file closes automatically on destruction)
    // -------------------------------------------------------------------
    std::ofstream logFile_;
};