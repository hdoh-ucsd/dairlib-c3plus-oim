#pragma once
#include <cstdlib>
#include <iostream>
#include <random>

/// Blending utilities
enum BLEND_FUNC { kSigmoid, kExponential };

double blend_sigmoid(double t, double tau, double window);

double blend_exp(double t, double tau, double window);

/// Selects the historical nondeterministic generator unless an explicit
/// experiment seed is present.  The deterministic stream persists across
/// sampler calls in the controller process.
inline std::mt19937& SamplingC3RandomGenerator(
    std::mt19937& nondeterministic_generator) {
    const char* seed_env = std::getenv("SAMPLING_C3_SEED");
    if (seed_env == nullptr || seed_env[0] == '\0') {
        return nondeterministic_generator;
    }

    static thread_local std::mt19937 deterministic_generator;
    static thread_local bool deterministic_seed_initialized = false;
    if (!deterministic_seed_initialized) {
        const auto seed = static_cast<std::mt19937::result_type>(
            std::strtoul(seed_env, nullptr, 10));
        deterministic_generator.seed(seed);
        deterministic_seed_initialized = true;
        std::cout << "[SAMPLER-SEED] deterministic seed=" << seed
                  << std::endl;
    }
    return deterministic_generator;
}

/// Random sampling utility.
inline double RandomUniform(double min, double max) {
    static thread_local std::mt19937 nondeterministic_rng{
        std::random_device{}()};
    std::mt19937& rng = SamplingC3RandomGenerator(nondeterministic_rng);
    std::uniform_real_distribution<double> dist(min, max);
    return dist(rng);
}

int FindBin(const double* bins, int n, double x);
