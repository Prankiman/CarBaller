#include "audio.h"

#include <SDL.h>

#include <cstdio>
#include <cstring>

// Per-sound mix gains, tuned by ear against the source levels (boost and
// motor are continuous loops, thud/jump/menu are one-shots).
static constexpr float kBoostGain = 0.45f;
static constexpr float kMotorGain = 0.30f;
static constexpr float kMenuGain = 0.70f;
static constexpr float kThudGain = 0.80f;
static constexpr float kJumpGain = 0.60f;

// Per-sample gain chase: ~26 ms time constant at 48 kHz. Loops fade in and
// out without clicks; one-shots start AT their target, so this never moves
// them and the attack of a hit stays intact.
static constexpr float kGainChase = 0.0008f;

bool Audio::init(const std::string& soundDir) {
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        std::fprintf(stderr, "[audio] subsystem unavailable: %s\n", SDL_GetError());
        return false;
    }
    subsystemUp_ = true;

    bool ok = true;
    if (!load(boost_, soundDir + "/boost.wav")) ok = false;
    if (!load(motor_, soundDir + "/motor.wav")) ok = false;
    if (!load(thud_, soundDir + "/thud.wav")) ok = false;
    if (!load(menu_, soundDir + "/menu.wav")) ok = false;
    if (!load(jump_, soundDir + "/jump.wav")) ok = false;
    (void)ok;   // misses are already logged; whatever loaded still plays

    SDL_AudioSpec want;
    std::memset(&want, 0, sizeof(want));
    want.freq = kRate;
    want.format = AUDIO_F32SYS;
    want.channels = kChannels;
    want.samples = 1024;
    want.callback = sdlCallback;
    want.userdata = this;

    // allowed_changes = 0: SDL hands the callback exactly this format and
    // converts to the hardware behind the scenes, so the mixer can assume
    // F32 / 48 kHz / stereo unconditionally.
    dev_ = SDL_OpenAudioDevice(nullptr, 0, &want, nullptr, 0);
    if (!dev_) {
        std::fprintf(stderr, "[audio] no playback device: %s (running silent)\n",
                     SDL_GetError());
        return false;
    }
    SDL_PauseAudioDevice(dev_, 0);
    std::fprintf(stderr, "[audio] ready: %s\n", soundDir.c_str());
    return true;
}

void Audio::shutdown() {
    if (dev_) {
        SDL_CloseAudioDevice(dev_);
        dev_ = 0;
    }
    if (subsystemUp_) {
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        subsystemUp_ = false;
    }
}

bool Audio::load(Sound& out, const std::string& path) {
    SDL_AudioSpec spec;
    Uint8* buf = nullptr;
    Uint32 len = 0;
    if (!SDL_LoadWAV(path.c_str(), &spec, &buf, &len)) {
        std::fprintf(stderr, "[audio] missing %s: %s\n", path.c_str(), SDL_GetError());
        return false;
    }

    // NB: SDL_NewAudioStream takes (format, channels, rate) pairs - the
    // channels argument comes BEFORE the rate.
    SDL_AudioStream* st = SDL_NewAudioStream(spec.format, spec.channels, spec.freq,
                                             AUDIO_F32SYS, kChannels, kRate);
    if (!st || SDL_AudioStreamPut(st, buf, (int)len) != 0 ||
        SDL_AudioStreamFlush(st) != 0) {
        std::fprintf(stderr, "[audio] convert failed for %s: %s\n", path.c_str(),
                     SDL_GetError());
        if (st) SDL_FreeAudioStream(st);
        SDL_FreeWAV(buf);
        return false;
    }
    SDL_FreeWAV(buf);

    const int bytes = SDL_AudioStreamAvailable(st);
    const int floats = bytes / (int)sizeof(float);
    out.samples.assign(floats > 0 ? floats : 0, 0.0f);
    if (floats > 0) SDL_AudioStreamGet(st, out.samples.data(), bytes);
    SDL_FreeAudioStream(st);

    out.frames = (int)out.samples.size() / kChannels;
    if (out.frames <= 0) {
        std::fprintf(stderr, "[audio] empty after convert: %s\n", path.c_str());
        return false;
    }
    return true;
}

void Audio::setMasterVolume(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    lock();
    master_ = v;
    unlock();
}

void Audio::setBoostLoop(bool active) {
    if (!dev_ || boost_.frames <= 0) return;
    lock();
    Voice& v = voices_[kBoostVoice];
    if (active) {
        if (!v.active) {           // fresh start (or resume after fade-out)
            v.snd = &boost_;
            v.loop = true;
            v.step = 1;
            v.cur = 0;             // fade in via the chase
        }
        v.target = kBoostGain;
        v.active = true;
    } else {
        v.target = 0;              // fade out; the callback retires the voice
    }
    unlock();
}

void Audio::setMotorLoop(bool active, float speed01) {
    if (!dev_ || motor_.frames <= 0) return;
    if (speed01 < 0.0f) speed01 = 0.0f;
    if (speed01 > 1.0f) speed01 = 1.0f;
    lock();
    Voice& v = voices_[kMotorVoice];
    v.step = 0.9f + 0.4f * speed01;   // idle .. supersonic pitch sweep
    if (active) {
        if (!v.active) {
            v.snd = &motor_;
            v.loop = true;
            v.cur = 0;
        }
        v.target = kMotorGain;
        v.active = true;
    } else {
        v.target = 0;
    }
    unlock();
}

void Audio::playMenu(float gain) { playOneShot(menu_, kMenuGain * gain); }
void Audio::playThud(float gain) { playOneShot(thud_, kThudGain * gain); }
void Audio::playJump(float gain) { playOneShot(jump_, kJumpGain * gain); }

void Audio::playOneShot(const Sound& s, float gain) {
    if (!dev_ || s.frames <= 0 || gain <= 0.0f) return;
    lock();
    int slot = -1;
    for (int i = kLoopVoices; i < kVoices; ++i) {
        if (!voices_[i].active) { slot = i; break; }
    }
    if (slot < 0) {   // all busy: steal round-robin (oldest of the one-shots)
        slot = kLoopVoices + (nextShot_ % (kVoices - kLoopVoices));
        ++nextShot_;
    }
    Voice& v = voices_[slot];
    v.snd = &s;
    v.pos = 0;
    v.step = 1;
    v.cur = v.target = gain;   // no fade - one-shots keep their attack
    v.loop = false;
    v.active = true;
    unlock();
}

void Audio::lock() { if (dev_) SDL_LockAudioDevice(dev_); }
void Audio::unlock() { if (dev_) SDL_UnlockAudioDevice(dev_); }

void Audio::sdlCallback(void* ud, Uint8* stream, int len) {
    static_cast<Audio*>(ud)->mix(stream, len);
}

void Audio::mix(Uint8* stream, int len) {
    float* out = reinterpret_cast<float*>(stream);
    const int frames = len / (int)(sizeof(float) * kChannels);
    std::memset(stream, 0, len);

    for (auto& v : voices_) {
        if (!v.active || !v.snd || v.snd->frames <= 0) continue;
        const float* d = v.snd->data();
        const int F = v.snd->frames;

        for (int i = 0; i < frames; ++i) {
            if (v.target == 0.0f && v.cur < 0.0005f) { v.active = false; break; }
            if (!v.loop && v.pos >= F) { v.active = false; break; }

            int idx = (int)v.pos;
            if (idx >= F) {          // loop wrap; non-loop exits above
                v.pos -= F;
                idx = 0;
            }
            const float frac = (float)(v.pos - idx);
            const int n1 = (idx + 1 < F) ? idx + 1 : (v.loop ? 0 : idx);

            const float sl = d[idx * 2] + (d[n1 * 2] - d[idx * 2]) * frac;
            const float sr = d[idx * 2 + 1] + (d[n1 * 2 + 1] - d[idx * 2 + 1]) * frac;

            v.cur += (v.target - v.cur) * kGainChase;
            out[i * 2] += sl * v.cur;
            out[i * 2 + 1] += sr * v.cur;
            v.pos += v.step;
        }
    }

    const float m = master_;
    const int n = frames * kChannels;
    for (int i = 0; i < n; ++i) {
        const float s = out[i] * m;
        out[i] = s < -1.0f ? -1.0f : (s > 1.0f ? 1.0f : s);
    }
}
