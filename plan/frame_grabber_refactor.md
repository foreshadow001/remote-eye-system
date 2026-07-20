# Refactor: FrameGrabber Library

## Motivation

`test_multi_cam_multi_host.cpp` ~1200 lines. Monolithic: camera threads, recording, network sync, UI rendering, and config loading all in one file. Maintenance cost is high — every bug fix or feature added to camera acquisition must be replicated across `test_calib_images.cpp`, `test_record_arm_data.cpp`, etc.

## Feasibility: YES

The building blocks already exist in `/utils/cam/` (`BaslerCamera`, `FrameMeta`, `FrameCallback`). The refactoring extracts the common acquisition pipeline into a reusable `FrameGrabber` class, keeping the test scripts responsible only for config loading, UI, and user interaction.

## Proposed Architecture

```
┌──────────────────────────────────────────────────────┐
│ test_multi_cam_multi_host.cpp  (~300 lines)          │
│  - Config loading                                     │
│  - OpenCV UI (thumbnails + enlarged view)             │
│  - Keyboard handler (r, space, q)                     │
│  - NetworkSync setup (master/slave UDP)               │
│  - Health monitor callback                            │
│  - Recording callback (on-frame, on-complete)         │
└──────────┬───────────────────────────────────────────┘
           │ uses
┌──────────▼───────────────────────────────────────────┐
│ utils/cam/FrameGrabber          (~400 lines)          │
│  - CameraContext (internal)                           │
│  - captureWorker + copyWorker per camera              │
│  - RAM pre-allocation + streaming                     │
│  - instantTrigger()                                   │
│  - dumpToDisk() + convertToJpg()                      │
│  - Health monitoring (last_block_id, last_frame_time) │
│  - Fault detection + restart                          │
│  - Public API:                                         │
│      open(cam_sns, fps, gain, gamma, exp)             │
│      startRecording(n_frames)                         │
│      getLatestFrame(cam_idx) → cv::Mat                │
│      getHealth(cam_idx) → {block_id, streaming, age}  │
│      onFrameDropped → callback                        │
│      onRecordingComplete → callback                   │
│      close()                                           │
└──────────────────────────────────────────────────────┘
           │ uses
┌──────────▼───────────────────────────────────────────┐
│ utils/cam/NetworkSync            (~150 lines)          │
│  - Master/slave UDP socket management                 │
│  - CMD_START broadcast / listen                       │
│  - Fault message send / receive                       │
│  - SHUTDOWN signal                                     │
│  - Public API:                                         │
│      init(role, local_ip, peer_ip, port)              │
│      broadcastStart()                                 │
│      waitForStart() → async callback                  │
│      sendFault(cam_idx)                               │
│      pollFault() → optional<fault_info>                │
│      sendShutdown()                                    │
└──────────────────────────────────────────────────────┘
           │ uses
┌──────────▼───────────────────────────────────────────┐
│ utils/cam/BaslerCamera (existing)                     │
│  - Pylon SDK wrapper                                  │
│  - open/close/start, setFrameRate/Gain/Gamma/Exposure │
│  - FrameCallback → (GrabResultPtr, FrameMeta)         │
│  - isMono(), getSerialNumber()                        │
└──────────────────────────────────────────────────────┘
```

## FrameGrabber Public API

```cpp
class FrameGrabber {
public:
    struct Config {
        std::vector<std::string> camera_sns;
        double fps, gain, gamma, exposure_time;
        bool hw_trigger;
        bool enable_offset;
        int cam_width, cam_height;
    };

    explicit FrameGrabber(const Config& cfg);

    // Lifecycle
    bool open();                          // Create capture+copy threads for all cameras
    void close();                         // Join all threads, release resources
    int  cameraCount() const;

    // Live preview (non-blocking, thread-safe shallow copies)
    cv::Mat latestFrame(int cam_idx) const;
    FrameMeta latestMeta(int cam_idx) const;
    bool isMono(int cam_idx) const;
    std::string cameraSn(int cam_idx) const;

    // Recording
    void prepareRecording(int total_frames);   // Pre-allocate RAM buffers
    void startRecording();                     // Signal all cameras to start streaming to RAM
    bool isRecording() const;
    int  recordedFrames(int cam_idx) const;   // Per-camera progress
    bool allDumpReady() const;                // All cameras reached target

    // Post-recording
    void dumpToDisk(const std::string& temp_dir, const std::string& log_path,
                    int write_delay_ms);       // Blocking per camera
    void convertToJpg(const std::string& raw_dir, const std::string& jpg_dir,
                      const std::vector<LogEntry>& entries, bool mono); 

    // Health
    struct Health { int64_t last_block_id; bool streaming; double age_seconds; };
    Health health(int cam_idx) const;
    bool anyStalled(double timeout_sec) const;

    // Callbacks
    using FrameCallback = std::function<void(int cam_idx, const cv::Mat& frame)>;
    using FaultCallback  = std::function<void(int cam_idx)>;
    void setFrameCallback(FrameCallback cb);
    void setFaultCallback(FaultCallback cb);
};
```

## NetworkSync Public API

```cpp
class NetworkSync {
public:
    enum Role { Master, Slave };
    struct Config {
        Role role;
        std::string local_ip, peer_ip;
        int port;
    };

    explicit NetworkSync(const Config& cfg);
    bool init();

    // Recording trigger
    void broadcastStart();                  // Master: send CMD_START
    bool waitForStart(int timeout_ms);      // Slave: block until CMD_START

    // Fault
    void sendFault(int cam_idx);
    struct FaultInfo { bool from_master; int cam_idx; };
    std::optional<FaultInfo> pollFault();   // Non-blocking

    // Shutdown
    void sendShutdown();
    bool shutdownReceived();

    void close();
};
```

## Test Script Refactoring

`frame_grabber.cpp` (~300 lines, replaces `test_multi_cam_multi_host.cpp`):

```cpp
int main() {
    Cfg cfg;
    auto grabber_cfg = loadGrabberConfig(cfg["test_multi_cam"]);
    auto net_cfg     = loadNetSyncConfig(cfg["test_multi_cam"]);

    FrameGrabber grabber(grabber_cfg);
    NetworkSync  net(net_cfg);
    grabber.open();
    net.init();

    // UI setup
    while (global_running) {
        // Keyboard: r → net.broadcastStart(); grabber.startRecording()
        // Keyboard: space → grabber.latestFrame().clone() → imwrite

        // UI render: grabber.latestFrame(i) for each thumbnail
        // UI render: grabber.health(i) → green/red indicator

        // Net poll: net.pollFault() → show fault UI
        // Grabber health: grabber.anyStalled() → net.sendFault()

        // Recording complete: grabber.allDumpReady() → dump → convert
    }

    // Cleanup
    grabber.close();
    net.close();
}
```

## Benefits

| Before | After |
|--------|-------|
| 1200-line monolith | 300-line test + 400-line library + 150-line network |
| Camera logic duplicated in 3+ test files | One `FrameGrabber` used everywhere |
| Health check interleaved with UI code | `Health` struct via getter |
| Network sync interleaved with camera code | Independent `NetworkSync` class |
| Recording logic spread across globals | Encapsulated in `FrameGrabber` |

## Implementation Order

1. **`FrameGrabber`** — extract CameraContext, captureWorker, copyWorker, recording, health
2. **`NetworkSync`** — extract UDP master/slave, CMD_START, fault, shutdown
3. **`frame_grabber.cpp`** — new test using both classes + OpenCV UI
4. **Regression** — apply FrameGrabber to `test_calib_images.cpp`, `test_record_arm_data.cpp`

## Atomic Globals → Encapsulation

Current globals (~15 atomics) and how they're absorbed:

| Current Global | Moved To | Access Pattern |
|----------------|----------|----------------|
| `global_running` | Shared `atomic<bool>*` passed to `FrameGrabber` | `grabber.close()` sets false → all threads exit |
| `net_cmd_record` | `NetworkSync` internal | `waitForStart(timeout)` returns `bool` |
| `g_fault_active`, `g_faulty_cam`, `g_fault_on_master` | `FrameGrabber` internal `FaultState` struct | `grabber.faultState()` → snapshot |
| `g_fault_time`, `g_ready_time` | `FrameGrabber` internal | `grabber.uptime()` / `grabber.faultUptime()` |
| `g_xfer_active` | N/A (transfer logic stays in test scripts) | — |
| `g_enlarged_cam` | `MultiCamPreview` internal | mouse callback sets, render reads |
| `g_last_capture_index` | Test script local | not in library |
| `CameraContext::recording`, `dump_ready`, `running` | `FrameGrabber::CameraSlot` (private struct) | exposed via `isRecording()`, `isDumpReady()` |
| `shared_record_timestr` | Test script local | passed to `FrameGrabber::dumpToDisk()` |
| `is_recording`, `is_dumping` | Test script local `bool` | derived from `grabber.isRecording()` |

**Pattern**: `FrameGrabber` takes a `const atomic<bool>& running` reference at construction. All internal state is protected by mutex or atomic members — no global scope leakage.

## OpenCV UI Encapsulation → `MultiCamPreview`

Reusable class for the 5×2 thumbnail + enlarged view pattern.

```cpp
class MultiCamPreview {
public:
    struct Config {
        std::string window_name;
        int window_w, window_h;
        double ui_fps;
    };

    explicit MultiCamPreview(const Config& cfg);

    // Feed frames from grabber
    void updateFrame(int slot, const cv::Mat& bgr, const std::string& label);
    void setStatus(int slot, const std::string& status_msg); // "REC", "ERROR", etc.

    // Selection
    int  selectedSlot() const;          // -1 = none
    void onMouse(int event, int x, int y);

    // Overlay (fault / transfer / recording banner)
    void showOverlay(const std::string& title, const std::vector<std::string>& lines);

    // Main loop
    char waitKeyEx(int ms);             // returns key, renders on timer
    void render();                       // force re-render

private:
    // Internal layout: left panel (5×2 grid), right panel (enlarged)
    int thumbW_, thumbH_, leftW_, rightX_, rightW_;
    std::vector<cv::Mat> frames_;         // slot → BGR thumbnail source
    std::vector<std::string> labels_;
    int selected_;
    bool show_overlay_;
    std::string overlay_title_;
    std::vector<std::string> overlay_lines_;
    // timing
    std::chrono::steady_clock::time_point last_render_;
    int render_interval_ms_;
};
```

**Usage in test script**:

```cpp
MultiCamPreview preview({"Multi-Cam Preview", 1600, 800, 20.0});

while (running) {
    // Feed latest frames
    for (int i = 0; i < grabber.cameraCount(); ++i)
        preview.updateFrame(i, grabber.latestFrameBgr(i), grabber.cameraSn(i));

    // Keyboard
    char key = preview.waitKeyEx(1);
    if (key == 'r') grabber.startRecording();
    // ...

    // Fault
    if (grabber.anyStalled(1.0)) {
        preview.showOverlay("CAMERA FAULT", {grabber.faultInfo()});
    }
}
```

This eliminates the 4 rendering functions, mouse callback, layout computation, and overlay logic currently duplicated across `test_multi_cam_multi_host.cpp` and `test_calib_images.cpp`.

## Risk

- **Low**: FrameGrabber reuses existing `BaslerCamera`, threading model unchanged
- **Low**: `MultiCamPreview` is a pure refactor of existing rendering code — no new rendering logic
- **Medium**: 3 existing test files need updating to use new API (or keep old ones as-is, deprecate over time)
- **Verdict**: Feasible, recommended
