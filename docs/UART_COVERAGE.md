# UART coverage — development branch

Reference: [SEN0677 protocol V1.0](https://dfimg.dfrobot.com/wiki/21122/SEN0677_ai-binocular-face-palm-vein-qr-code-recognition-sensor_datasheet_V1.0.pdf).

All changes below are experimental until compiled and verified on physical hardware. The vendor PDF includes several module families. The existing QR payload is preserved by default; opt into the documented format with `documented_qr: true`.

| Command | Development status |
|---|---|
| 10 reset | Implemented; stop workaround retained, verification replies suppressed |
| 11 status | Query and getter |
| 12 verify / auto verify | Existing API + optional max attempts overload |
| 13 directional enroll | Experimental method; direction and completion semantics require validation |
| 1D single enrollment | Existing API + outcome events |
| 20 / 21 delete | Existing API, user list invalidated |
| 22 user info | Query + event |
| 23 clear enrollment state | Method |
| 24 all UIDs | Aggregated pages; completes on page <100; count is 16-bit |
| 26 integrated enrollment | Experimental method, caller supplies documented mode |
| 30 version / 93 serial | Queries and getters |
| 70 QR | Legacy and documented request variants; QR no longer sets biometric authorization |
| B0 / B1 UVC | Experimental query/configuration; query result logged |
| 71 / 72 / 74 image | Experimental opt-in streaming callback; no HA camera entity |
| 73 enroll captured face | Not implemented: depends on capture and duplicate policy |
| D7 / F7 photo enroll | F7 experimental upload; D7 not implemented |
| FA / FB templates | Experimental opt-in streaming export / bounded import; caller owns secure storage |
| 50 / 52 / 53 encryption | Not implemented: verify cipher/key negotiation and recovery |
| F6 firmware upgrade | Not implemented: explicit recovery plan required |
| FC duplicate check | Method; leave duplicate checking enabled for access control |
| FE demo | Deliberately not exposed for access control: bypasses biometric comparison |

The command echo is not a transaction ID. Delayed replies from an earlier instance of the same command cannot always be distinguished. Do not deploy experimental firmware as the only authorization mechanism on an occupied door before hardware validation.

## New automation events
```yaml
dfrobot_ai10:
  id: ai10_sensor
  uart_id: uart_ai10
  on_enrollment_success:
    then:
      - logger.log:
          format: "Enrollment complete: %u"
          args: [uid]
  on_enrollment_failed:
    then:
      - logger.log:
          format: "Enrollment failed: %u"
          args: [result]
  on_recognition_failed:
    then:
      - logger.log:
          format: "Recognition failed: %u"
          args: [result]
  on_users_received:
    then:
      - logger.log:
          format: "User count: %u"
          args: [count]
```
Other events: `on_command_result(command, result)`, `on_user_info(uid, name, admin)`. Call `query_user(uid)` for each desired ID after list completion. User listing does not automatically restart recognition. Use `get_user_ids()`, `is_user_list_complete()`, `get_module_status()`, `get_firmware_version()`, and `get_serial_number()` in lambdas. Call `start_recognition(timeout, continuous, max_attempts)` to opt into the extended verification request.

No new hardware results are available. The upstream stop workaround is retained because RESET reportedly does not reliably terminate automatic recognition.

## Sensitive transfers (experimental C++ API only)
Set `allow_sensitive_transfers: true` to use these explicit methods; never publish image or template bytes to logs or Home Assistant text sensors. Downloads require a caller-supplied synchronous chunk sink returning true on success, false on storage failure. The callback must be fast, must not re-enter the component, and must copy data before returning. Completion is reported by `on_command_result`. Capture size is bounded to 255 chunks, template size to 16 KiB; photo upload to 64 KiB. Template imports with lengths exactly divisible by 246 are rejected pending clarification of termination semantics. A failing transfer may have already written partial data or changed module state: discard incomplete backups and inspect the user database before retrying. No built-in storage, encryption or automatic upload is provided.

The vendor manual describes a proprietary session-key derivation for a different module family, not enough to implement interoperable encryption. No key-writing commands or demo bypass are exposed. Physical testing is mandatory.
## Hardware observations and user directory
Raw UART replies on AI-10 firmware v2.30-QRXF-PV confirm commands 0x11, 0x22, 0x24, 0x30, 0x93 and 0xB0. The 0x22 response is 35 bytes (2 UID, 32 name, 1 admin), for both face and palm and both admin flag values. The observed 0x24 reply contains six IDs in one page; pagination has not been hardware-tested. The serial response has 32 bytes, with 0xFF padding after the printable serial. Store the raw response in `get_serial_raw()`, and expose only printable prefix in `get_serial_number()`. These measurements validate the protocol, not the new ESPHome implementation. No real names or serial are included in fixtures.

Call `refresh_users()` to get UID pages and then each user's record in sequence. The completed directory is available through `get_users()` and `is_user_directory_complete()`. `on_user_info` fires for each record; `on_user_directory(count)` signals completion and `on_user_directory_failed(result)` reports a timeout or failure. No partially collected directory is published as complete. Opt into automatic refresh after successful enrollment/deletion with `auto_refresh_users: true`. Opt into restoration of a previously active continuous recognition mode after a successful refresh with `resume_recognition_after_refresh: true`. Both options default off. Recognition stays stopped on a failed refresh. Stop/reset cancels an active refresh. Do not use the truncated text summary as an authorization database. See [user directory example](../examples/user-directory.yaml).

## Manual snapshot camera (experimental)
The optional `dfrobot_ai10_camera` component exposes a native ESPHome camera entity containing the last manually captured JPEG (0x71, documented 240x320). It requires an explicit `allow_sensitive_transfers: true` in the parent and a separate camera config. See [example](../examples/snapshot-camera.yaml). Stop recognition first, wait for the stop workaround to finish, then call `take_snapshot()`. HA image requests and stream requests only serve the cached still; they never start capture or interrupt authentication. Start recognition manually afterwards. The default JPEG size limit is 48 KiB (configurable to 64 KiB). The previous image is released before new capture and a busy reader prevents replacement. JPEG SOI/EOI and exact transfer length are checked; actual format and command support still require hardware verification. No video stream, persistent storage, automatic capture, or public HTTP server is provided. The image is sensitive; use an encrypted ESPHome API and trusted Home Assistant access. Do not enable raw UART dumps during image transfer. No photos are committed to this repository.

## Failed recognition archive support (experimental)
The camera now exposes `take_face_snapshot()` (native 0x72, 320x320; requires visible frontal face, first reply limited to 8 seconds) in addition to the tested 0x71 path. A successful face snapshot does not trigger recognition callbacks or authorize unlocking, even if the module returns a matching UID. `last_capture_succeeded()` distinguishes a fresh successful capture from a failed or refused request. `hold_for_archive(token)` protects the current JPEG until `acknowledge_archive(token)` receives the matching nonempty token. All manual and automated capture requests refuse to overwrite a held JPEG. Archive protection is RAM-only; a power cycle loses the pending image. This interface permits ESPHome to restart recognition before HA has written the photo to disk. A failed HA disk write should not acknowledge the token. No face image or disk archive has yet been hardware-tested on this revision.

## Compact door package
The C++ camera component now owns the failed-recognition counter, deferred stop/capture/fallback/restart sequence, archive protection, and retry scheduling. It counts only 0x08 and 0x0C from active continuous verification; success clears the count. It retains the actual recognition timeout/attempt settings for restart. Explicit stop in the package cancels the sequence. An optional `on_archive_request(token)` trigger calls HA via its native API, with `archive_result(token, success)` acknowledging only the matching cached image. No separate HA automation is required. The package requires ESPHome 2025.10+ and permission to perform HA actions; it writes timestamped unique names under /media by default. An uncertain acknowledgement may lead to duplicate files, never to replacing a different event's image. Pending archives are RAM-only; further capture is blocked until confirmed. The stock package is `packages/ai10-door.yaml`; it retains regularly used entities and drops one-time diagnostic buttons. Camera entity ID must match the HA instance. Real face captures and the archive handshake need physical validation.
