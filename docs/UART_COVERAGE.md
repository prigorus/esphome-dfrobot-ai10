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