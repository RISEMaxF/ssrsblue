import time

from fastapi import APIRouter, HTTPException, Request

from connector.models import GPSStatusResponse

router = APIRouter(tags=["gps"])


@router.get("/gps", response_model=GPSStatusResponse)
async def get_gps(request: Request) -> GPSStatusResponse:
    gps_state = getattr(request.app.state, "gps_state", None)
    if gps_state is None:
        raise HTTPException(status_code=404, detail="GPS reader not enabled")

    return GPSStatusResponse(
        enabled=True,
        serial_connected=gps_state.serial_connected,
        fix_quality=gps_state.fix_quality,
        lat=gps_state.lat,
        lon=gps_state.lon,
        altitude=gps_state.altitude,
        satellites=gps_state.satellites,
        hdop=gps_state.hdop,
        speed_knots=gps_state.speed_knots,
        course=gps_state.course,
        utc_time=gps_state.utc_time,
        last_sentence_age_s=round(time.monotonic() - gps_state.last_sentence_time, 1),
        sentences_received=gps_state.sentences_received,
        parse_errors=gps_state.parse_errors,
    )
