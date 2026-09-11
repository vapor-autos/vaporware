TURBO_STEER_ASSIST_STATE_SERVICE = "turboSteerAssistState"


def is_turbo_steer_override_active(sm, started: bool) -> bool:
  selfdrive_service = "selfdriveState"
  assist_service = TURBO_STEER_ASSIST_STATE_SERVICE
  return bool(
    started
    and sm.seen[selfdrive_service]
    and sm.alive[selfdrive_service]
    and sm.valid[selfdrive_service]
    and sm[selfdrive_service].enabled
    and sm.seen[assist_service]
    and sm.alive[assist_service]
    and sm.valid[assist_service]
    and sm[assist_service].applied
  )
