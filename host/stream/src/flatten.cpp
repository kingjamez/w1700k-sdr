// SPDX-License-Identifier: MIT
#include "flatten.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/stat.h>
#include <unistd.h>

namespace {

// The decimation's roll-off per sample rate: dB at |f| = k/64 of Nyquist, k = 0..64. Measured on
// unit A, band 1 (MT7977), from the receiver's own noise floor (which the roll-off attenuates as it
// does a signal: ledger D293, D296), analog filter as wide as the sample rate, LNA 2 / baseband 5;
// relative to 15..45 % of Nyquist, both sides averaged, 0 dB below 45 %, made monotonic and
// lightly smoothed. The 640/2 rate is flat to about 0.97 x Nyquist and has none.
constexpr unsigned kPoints = 65;
constexpr float kRolloff40[kPoints] = {
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,  -0.6f,  -1.1f,  -1.3f,  -1.5f,  -1.8f,  -2.1f,  -2.4f,  -2.7f,  -3.1f,  -3.5f,
     -3.9f,  -4.4f,  -4.9f,  -5.4f,  -5.9f,  -6.5f,  -7.2f,  -7.8f,  -8.5f,  -9.2f, -10.0f, -10.7f, -11.6f,
    -12.4f, -13.3f, -14.2f, -15.1f, -16.0f, -16.9f, -17.7f, -18.5f, -19.2f, -19.8f, -20.2f, -20.5f, -20.7f,
};
constexpr float kRolloff80[kPoints] = {
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,  -0.4f,  -0.7f,  -0.9f,  -1.1f,  -1.3f,  -1.6f,  -1.9f,  -2.2f,  -2.6f,  -3.0f,
     -3.4f,  -3.8f,  -4.3f,  -4.8f,  -5.3f,  -5.9f,  -6.5f,  -7.1f,  -7.8f,  -8.5f,  -9.3f, -10.1f, -11.0f,
    -11.8f, -12.8f, -13.8f, -14.8f, -15.9f, -17.0f, -18.2f, -19.3f, -20.5f, -21.5f, -22.4f, -23.1f, -23.4f,
};
constexpr float kRolloff160[kPoints] = {
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,  -0.3f,  -0.6f,  -0.8f,  -1.0f,  -1.2f,  -1.5f,  -1.7f,  -2.1f,  -2.4f,  -2.8f,
     -3.1f,  -3.6f,  -4.1f,  -4.6f,  -5.1f,  -5.7f,  -6.3f,  -6.9f,  -7.6f,  -8.4f,  -9.1f,  -9.9f, -10.8f,
    -11.7f, -12.7f, -13.7f, -14.8f, -15.9f, -17.1f, -18.4f, -19.7f, -21.0f, -22.3f, -23.4f, -24.2f, -24.7f,
};
constexpr float kRolloff320[kPoints] = {
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f,
      0.0f,   0.0f,   0.0f,  -0.1f,  -0.2f,  -0.3f,  -0.5f,  -0.6f,  -0.8f,  -1.1f,  -1.4f,  -1.7f,  -2.0f,
     -2.4f,  -2.8f,  -3.2f,  -3.6f,  -4.1f,  -4.6f,  -5.1f,  -5.7f,  -6.3f,  -6.9f,  -7.6f,  -8.4f,  -9.2f,
    -10.1f, -11.0f, -12.0f, -13.0f, -14.0f, -15.2f, -16.4f, -17.8f, -19.1f, -20.5f, -21.7f, -22.6f, -23.1f,
};

constexpr unsigned kDesignPoints = 8192;  // frequency-sampling grid of the FIR design
constexpr double kKaiserBeta = 4;         // window of the 129 taps (<= 0.15 dB error to 0.95 x Nyquist)

// The kFft-point transforms, planned once per process; every FlattenRun runs them on its own
// buffer (fftwf_execute_dft: in place, on arrays from fftwf_alloc_complex, aligned as the one
// planned on). FFTW_MEASURE times candidate plans: on x86 by the cycle counter, 0.1 s per
// transform, but Ubuntu's arm64 FFTW has none and times them by gettimeofday, 25 s per transform
// on a DGX Spark. So the service starts with measured plans only from wisdom an earlier run saved,
// otherwise with estimated ones (on the Spark 1.5 times as slow) while a thread measures: its
// plans replace them, and its wisdom is saved for the next start. Plans are never destroyed: a
// worker may still be running the old ones.
struct Plans {
    fftwf_plan forward, inverse;
};
std::atomic<const Plans *> plans_in_use{nullptr};
fftwf_plan transfer_plan;  // Flatten's, out of place and estimated as before: the same transfer

// $XDG_CACHE_HOME/w1700k-stream/fftwf-wisdom, by default under ~/.cache; empty without a home.
std::string wisdom_path()
{
    const char *cache = std::getenv("XDG_CACHE_HOME"), *home = std::getenv("HOME");
    if (cache && cache[0] == '/') return std::string(cache) + "/w1700k-stream/fftwf-wisdom";
    if (home && home[0] == '/') return std::string(home) + "/.cache/w1700k-stream/fftwf-wisdom";
    return "";
}

const Plans *make_plans(unsigned flags)
{
    fftwf_complex *buffer = fftwf_alloc_complex(Flatten::kFft);
    fftwf_plan forward = fftwf_plan_dft_1d(int(Flatten::kFft), buffer, buffer, FFTW_FORWARD, flags);
    fftwf_plan inverse = fftwf_plan_dft_1d(int(Flatten::kFft), buffer, buffer, FFTW_BACKWARD, flags);
    fftwf_free(buffer);
    if (forward && inverse) return new Plans{forward, inverse};
    fftwf_destroy_plan(forward);
    fftwf_destroy_plan(inverse);
    return nullptr;
}

// The only planner call after the first plans (FFTW's planner is not thread-safe; running plans
// is). The wisdom is written aside and renamed over the old: another service may be saving its own.
void measure(const std::string &path)
{
    const Plans *measured = make_plans(FFTW_MEASURE);
    if (!measured) return;
    plans_in_use.store(measured, std::memory_order_release);
    if (path.empty()) return;
    const std::string dir = path.substr(0, path.rfind('/')), temp = path + "." + std::to_string(getpid());
    mkdir(dir.substr(0, dir.rfind('/')).c_str(), 0700);
    mkdir(dir.c_str(), 0700);
    if (!fftwf_export_wisdom_to_filename(temp.c_str()) || std::rename(temp.c_str(), path.c_str()))
        std::remove(temp.c_str());
}

const Plans *plans()
{
    static std::once_flag planned;
    std::call_once(planned, [] {
        const std::string path = wisdom_path();
        if (!path.empty()) fftwf_import_wisdom_from_filename(path.c_str());
        fftwf_complex *in = fftwf_alloc_complex(Flatten::kFft), *out = fftwf_alloc_complex(Flatten::kFft);
        transfer_plan = fftwf_plan_dft_1d(int(Flatten::kFft), in, out, FFTW_FORWARD, FFTW_ESTIMATE);
        fftwf_free(in);
        fftwf_free(out);
        const Plans *p = make_plans(FFTW_MEASURE | FFTW_WISDOM_ONLY);
        const bool measured = p != nullptr;
        if (!p) p = make_plans(FFTW_ESTIMATE);
        if (!p || !transfer_plan) throw std::runtime_error("FFTW could not plan the correction's transforms");
        plans_in_use.store(p, std::memory_order_release);
        if (!measured) std::thread(measure, path).detach();
    });
    return plans_in_use.load(std::memory_order_acquire);
}

double interpolate(const float *table, double x)
{
    x = std::min(std::max(x, 0.0), 1.0) * (kPoints - 1);
    const unsigned k = std::min(unsigned(x), kPoints - 2);
    return table[k] + (x - k) * (table[k + 1] - table[k]);
}

double bessel_i0(double x)
{
    double sum = 1, term = 1;
    for (int k = 1; k < 50; ++k) {
        term *= (x / (2 * k)) * (x / (2 * k));
        sum += term;
    }
    return sum;
}

}  // namespace

const Flatten *Flatten::for_rate(double sample_rate)
{
    static std::mutex m;
    static std::map<long long, std::unique_ptr<Flatten>> made;
    const float *table = nullptr;
    const long long rate = std::llround(sample_rate);
    if (rate == 40000000) table = kRolloff40;
    else if (rate == 80000000) table = kRolloff80;
    else if (rate == 160000000) table = kRolloff160;
    else if (rate == 320000000) table = kRolloff320;
    if (!table) return nullptr;
    std::lock_guard<std::mutex> lock(m);
    auto &f = made[rate];
    if (!f) f.reset(new Flatten(double(rate), table));
    return f.get();
}

// Frequency sampling: the inverse of the roll-off (capped) on a fine grid, back to the time
// domain, the kTaps around zero kept under a Kaiser window. The gain is real and even, so the taps
// are real and symmetric: linear phase, delay kDelay.
Flatten::Flatten(double rate, const float *rolloff) : rate_(rate), rolloff_(rolloff), taps_(kTaps)
{
    std::vector<double> gain(kDesignPoints), impulse(kDesignPoints, 0.0);
    const double cap = std::pow(10.0, kMaxGainDb / 20);
    for (unsigned k = 0; k < kDesignPoints; ++k) {
        const double f = k < kDesignPoints / 2 ? double(k) / kDesignPoints : double(k) / kDesignPoints - 1;
        gain[k] = std::min(std::pow(10.0, -interpolate(rolloff, std::fabs(f) / 0.5) / 20), cap);
    }
    // The inverse DFT of a real even sequence: h[n] = (1/K) sum_k G[k] cos(2 pi k n / K).
    for (unsigned n = 0; n <= kDelay; ++n) {
        double sum = 0;
        for (unsigned k = 0; k < kDesignPoints; ++k) sum += gain[k] * std::cos(2 * M_PI * double(k) * n / kDesignPoints);
        impulse[n] = sum / kDesignPoints;
    }
    const double i0 = bessel_i0(kKaiserBeta);
    for (unsigned t = 0; t < kTaps; ++t) {
        const int n = int(t) - int(kDelay);
        const double r = double(n) / kDelay;
        taps_[t] = impulse[unsigned(std::abs(n))] * bessel_i0(kKaiserBeta * std::sqrt(std::max(0.0, 1 - r * r))) / i0;
    }
    // The transfer of the causal taps h[0..kTaps) for overlap-save, scaled by 1/kFft.
    transfer_.reset(fftwf_alloc_complex(kFft));
    fftwf_complex *in = fftwf_alloc_complex(kFft);
    for (unsigned k = 0; k < kFft; ++k) {
        in[k][0] = k < kTaps ? float(taps_[k] / kFft) : 0.0f;
        in[k][1] = 0;
    }
    plans();  // transfer_plan is made with them
    fftwf_execute_dft(transfer_plan, in, transfer_.get());
    fftwf_free(in);
}

double Flatten::rolloff_db(double x) const { return interpolate(rolloff_, x); }

double Flatten::gain_db(double x) const
{
    const double w = M_PI * std::min(std::max(x, 0.0), 1.0);  // radians/sample
    double re = 0;
    for (unsigned t = 0; t < kTaps; ++t) re += taps_[t] * std::cos(w * (double(t) - kDelay));
    return 20 * std::log10(std::fabs(re));
}

FlattenRun::FlattenRun()
{
    plans();  // planned now, not with the first correction
    buffer_ = fftwf_alloc_complex(Flatten::kFft);
}

FlattenRun::~FlattenRun() { fftwf_free(buffer_); }

// Overlap-save: transform kFft inputs, multiply by the taps' transfer, transform back; the last
// kBlock outputs are free of wrap-around. Output t (0..n) is the full convolution at t + kTaps - 1,
// i.e. centred on input t + kDelay: zero-phase against the caller's indexing.
void FlattenRun::apply(const Flatten &f, const float *in, size_t n, float *out)
{
    const fftwf_complex *h = f.transfer();
    const Plans *p = plans();
    const size_t total = n + Flatten::kTaps - 1;
    for (size_t j = 0; j < n; j += Flatten::kBlock) {
        for (unsigned k = 0; k < Flatten::kFft; ++k) {
            const size_t i = j + k;
            buffer_[k][0] = i < total ? in[2 * i] : 0.0f;
            buffer_[k][1] = i < total ? in[2 * i + 1] : 0.0f;
        }
        fftwf_execute_dft(p->forward, buffer_, buffer_);
        for (unsigned k = 0; k < Flatten::kFft; ++k) {
            const float re = buffer_[k][0] * h[k][0] - buffer_[k][1] * h[k][1];
            const float im = buffer_[k][0] * h[k][1] + buffer_[k][1] * h[k][0];
            buffer_[k][0] = re;
            buffer_[k][1] = im;
        }
        fftwf_execute_dft(p->inverse, buffer_, buffer_);
        const size_t m = std::min<size_t>(Flatten::kBlock, n - j);
        for (size_t k = 0; k < m; ++k) {
            out[2 * (j + k)] = buffer_[Flatten::kTaps - 1 + k][0];
            out[2 * (j + k) + 1] = buffer_[Flatten::kTaps - 1 + k][1];
        }
    }
}
