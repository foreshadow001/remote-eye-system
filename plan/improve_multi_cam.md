# Improve test_multi_cam_multi_host.cpp

## Current Architecture

| Module | Description |
|--------|-------------|
| `captureWorker` | Pylon callback → `copy_queue` (max 2 frames buffered) |
| `copyWorker` | Dequeue `copy_queue` → write to `ram_buffer` if recording, else update `latest_frame` |
| `instantTrigger` | Clear queue → set `recording=true`, Pylon callback writes to RAM from next frame |
| `dumpToDiskWorker` | After recording, write frames from `ram_buffer` to raw files on disk |
| `convertRawToJpgWorker` | raw → jpg conversion + BlockID intersection cropping |
| `udpListenerWorker` | Slave-side UDP listener for `CMD_START` trigger |
| UI thread | 5×2 thumbnail grid + enlarged view, 20fps |
| Health check | 1s no frame → fault handler |

## Existing Metrics (post-recording)

| Metric | Source | Granularity |
|--------|--------|-------------|
| Saved frames | `parseLogFile` → `logs.size()` | per-camera |
| Dropped frames | BlockID gap `>1` | per-camera |
| Actual FPS | `(n-1) / (last_ts - first_ts)` | per-camera |
| Disk dump time | `chrono` duration | global |
| Raw→JPG progress | `global_processed` atomic counter | global |

## Missing Metrics

```
┌─ Pre-recording ─────────────────────────────────────┐
│  Camera init latency      open()→start() timing      │
│  RAM pre-alloc time       10×N cv::Mat::zeros        │
└─────────────────────────────────────────────────────┘

┌─ During recording (real-time) ──────────────────────┐
│  Trigger delay            r press→first BlockID diff │
│  Cross-host sync          Master/Slave first BlockID  │
│  Inter-camera sync        Max BlockID diff same host  │
│  Queue pressure            copy_queue.size() peak     │
│  UI fps stability         cv::waitKey interval stats   │
│  RAM write rate            recorded_frames / elapsed  │
└─────────────────────────────────────────────────────┘

┌─ Post-recording ────────────────────────────────────┐
│  Disk write speed         MB/s per camera            │
│  Frame completeness       expected vs actual count    │
└─────────────────────────────────────────────────────┘
```

## Improvement Items

| # | Category | Change | Difficulty |
|---|----------|--------|------------|
| 1 | Metrics | Record `recording_start_blockID` for each camera at trigger moment → compute trigger latency | Low |
| 2 | Metrics | After recording, auto-print all above metrics to `report.txt` | Medium |
| 3 | Metrics | Real-time FPS + last frame timestamp in UI bottom-right | Low |
| 4 | Robustness | Log dropped-frame warning when `copy_queue` overflows (currently silent) | Low |
| 5 | Performance | Pre-touch `ram_buffer` pages after allocation to avoid page faults during recording | Low |
| 6 | Performance | Limit `convertRawToJpg` concurrency (use thread pool instead of unlimited threads) | Medium |
| 7 | Data Quality | Post-recording: verify actual frame count vs expected; warn if diff > 5% | Low |
| 8 | Debug | Print each camera's BlockID range (start-end) and timestamps after recording | Low |
| 9 | UI | Show per-camera drop indicator (red dot) on thumbnails during recording | Low |
| 10 | Network | Record network latency of `CMD_START` (master local time vs slave receive time) — requires protocol extension | Medium |

## Recommended Implementation Order

1. **Phase 1: #1, #3, #4, #5, #7, #8** — low difficulty, high value, no architecture changes
2. **Phase 2: #2** — aggregate report output
3. **Phase 3: #6, #9, #10** — require protocol or threading changes
