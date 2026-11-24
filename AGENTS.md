# Development instructions: SEN0677 / AI-10

## Goal
Extend this ESPHome external component to support the documented SEN0677 UART protocol, with a maintainable, non-blocking implementation and Home Assistant integration. Preserve the upstream MIT license and attribution.

## References
- Vendor protocol: https://dfimg.dfrobot.com/wiki/21122/SEN0677_ai-binocular-face-palm-vein-qr-code-recognition-sensor_datasheet_V1.0.pdf
- Original component: https://github.com/BobMcGlobus/esphome-dfrobot-ai10
- Vendor Arduino library: https://github.com/DFRobot/DFRobot_AI10

Treat the protocol document as the primary specification, but verify module- and firmware-specific behavior on hardware. Record disagreements between the document and the vendor library. Never claim hardware support solely because a command appears in the document.

## Workflow
- Keep main stable; develop on develop or short-lived feature branches based on develop.
- Preserve the existing component name, YAML schema, public method signatures and on_recognized/on_qr_scanned semantics where possible.
- Prefer small, reviewable changes, with documentation, tests and examples in the same change.
- Before declaring a feature complete, distinguish implementation, automated test, ESPHome compilation and physical hardware verification. Do not merge experimental changes into main without owner approval.

## Architecture and safety
- Separate packet framing, command serialization and reply parsing from the ESPHome-facing state machine as the project grows.
- Validate frame lengths, checksums, command echoes, payload bounds and result codes. Test truncated, corrupt, delayed, duplicate and unsolicited packets.
- Keep UART operations non-blocking; enforce bounded command timeouts, memory use and explicit busy/cancellation behavior. Avoid indefinite resets or blind command retries, particularly for enrollment and deletion.
- Target ESP32-C6 with ESP-IDF first; preserve compatibility with the existing ESP32-S3 example. Advanced image/crypto features must be optional and respect RAM constraints.
- Never signal successful authentication on a timeout, malformed frame, stale result, failed liveness check or communication loss. Only a validated, current successful recognition may trigger authorization.
- Avoid logging biometric templates, captured images, QR secrets or encryption keys. Provide explicit opt-in for sensitive data transfer and destructive operations; keep encryption keys and credentials out of this public repository.
- Maintain compatibility with existing sensor firmware until a documented change is validated. Do not guess undocumented illumination commands.

## Implementation roadmap
1. Establish baseline compilation (ESP32-C6 and ESP32-S3, ESP-IDF) and UART packet tests. Audit current implementation against the protocol, including optional fields and notifications.
2. Harden framing, timeouts, cancellation and response handling while preserving existing behavior.
3. Add enrollment result, recognition failure and communication error events, and meaningful result/status codes.
4. Add complete user management: individual user information, paginated user IDs, user count and structured results.
5. Add module status, firmware version and serial number queries, configurable recognition options and documented enrollment modes.
6. Audit QR scanning variants and remaining camera/configuration commands.
7. Implement optional image transfers, then biometric template import/export and encrypted UART communication where supported by this module. Handle firmware updates separately, with safeguards.
8. Publish a command coverage matrix (documented / implemented / compile-tested / hardware-tested / unsupported) and user-facing Home Assistant examples.

## Testing
Use deterministic byte-level fixtures from the protocol specification, test success and error paths, and compile example configurations without real Wi-Fi credentials. Require physical SEN0677 validation before declaring new commands hardware-tested. Keep examples safe for access-control deployments; never unlock solely from face presence or a failed recognition.
