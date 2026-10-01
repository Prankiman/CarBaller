#pragma once
// Sound effects through SDL2 core audio only - no extra dependency.
// Loads the WAVs from assets/sounds and mixes two persistent loops
// (boost, motor) plus one-shot voices (menu click, ball thud/collision,
// jump/flip) on the SDL audio callback thread. Every failure degrades
// to silence.

#include <SDL.h>

#include <string>
#include <vector>

class Audio {
public:
    // Loads all sounds and opens the playback device. Returns false when
    // audio is unavailable; every method then becomes a silent no-op.
    bool init(const std::string& soundDir);
    void shutdown();

    bool enabled() const { return dev_ != 0; }

    void setMasterVolume(float v);   // 0..1, applied to the final mix

    // Persistent loops (fade in/out in the callback, never click).
    void setBoostLoop(bool active);
    void setMotorLoop(bool active, float speed01);  // pitch scales with speed

    // One-shots (gain applied instantly to keep the attack).
    void playMenu(float gain = 1.0f);
    void playThud(float gain);       // 0..1, scaled by impact strength
    void playJump(float gain = 1.0f);  // jumps, double jumps and flips

private:
    struct Sound {
        std::vector<float> samples;  // interleaved stereo F32 at kRate
        int frames = 0;
        const float* data() const { return samples.data(); }
    };

    struct Voice {
        const Sound* snd = nullptr;
        double pos = 0;              // in frames; wraps when looping
        double step = 1;             // playback rate (pitch)
        float target = 0;            // gain this voice is heading for
        float cur = 0;               // gain chased per sample (~26 ms)
        bool loop = false;
        bool active = false;
    };

    static void sdlCallback(void* ud, Uint8* stream, int len);
    void mix(Uint8* stream, int len);
    bool load(Sound& out, const std::string& path);
    void playOneShot(const Sound& s, float gain);
    void lock();
    void unlock();

    static constexpr int kRate = 48000;
    static constexpr int kChannels = 2;
    static constexpr int kBoostVoice = 0;   // dedicated loop slots ...
    static constexpr int kMotorVoice = 1;
    static constexpr int kLoopVoices = 2;   // ... one-shots live above these
    static constexpr int kVoices = 16;

    Sound boost_, motor_, thud_, menu_, jump_;
    Voice voices_[kVoices];
    SDL_AudioDeviceID dev_ = 0;
    bool subsystemUp_ = false;
    float master_ = 0.7f;
    int nextShot_ = 0;
};
