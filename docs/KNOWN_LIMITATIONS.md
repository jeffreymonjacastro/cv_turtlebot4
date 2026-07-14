# Known Limitations

This project is best read as an evidence-driven robotics prototype, not as a completed autonomy product.

## Validation boundaries

- Offline synthetic replay does not model full robot dynamics, wheel slip, sensor latency, or floor conditions.
- Sector-level replay can expose controller choices but cannot prove physical collision avoidance.
- Dry-run validation confirms callbacks, event flow, and command gating, but not physical driving performance.
- Physical movement remains the final validation layer.

## Navigation limitations

- Recovery behavior still has stress cases with spin ratio and yaw saturation warnings.
- Dead-end and U-shaped scenarios remain difficult for a purely reactive stack.
- Real robot logs showed corner risk, side scrape risk, recovery loops, oscillation, spin, and yaw saturation intervals.
- Targeted turn/recovery captures did not include intentional `TURNING_LEFT` or `TURNING_RIGHT` states, so they did not validate the intended turn-controller path.
- The project intentionally does not present full Nav2/SLAM as the main solution.

## Perception limitations

- YOLO detections must pass freshness, confidence, area, center, debounce, and cooldown gates before affecting the FSM.
- Camera overlays can show detections that never become robot actions if event sync is stale or gates are mismatched.
- QR benchmarks should be expanded with negative cases and varied distance, angle, blur, lighting, and partial occlusion.
- Model weights may be local artifacts and may not be redistributable in every version of the repository.

## Systems limitations

- Robot-specific commands require local network configuration and ROS 2 setup.
- Legacy scripts exist for historical reference and may not follow the current architecture.
- Some generated evidence is ignored under `output/`; users may need a separate artifact bundle to inspect raw logs.
- The repository previously contained lab credentials in legacy notes. Current docs use placeholders, but history should be treated as potentially exposed unless rewritten privately.

## Future work

- Add a public, curated result artifact bundle with sanitized logs and summaries.
- Expand QR benchmark datasets with negatives and harder real-camera geometry.
- Add more real-log-derived replay cases for active turn states.
- Strengthen recovery behavior in dead ends and spin-trap scenarios.
- Add a configuration validator that flags unsafe or inconsistent runtime parameters before robot launch.
- Add CI for unit tests and deterministic replay summaries.
