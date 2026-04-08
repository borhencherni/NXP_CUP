#include "LineDetector.h"
#include "teensy/Config.h"
#include "usb_serial.h"

LineDetector::LineDetector(Pixy2& pixy) : _pixy(pixy) {}

// ─── Public ───────────────────────────────────────────────────────────────────

bool LineDetector::update() {
    _trackInfo = Sensors_Scan(_pixy);
    if (!_trackInfo.hasRight && !_trackInfo.hasLeft) return false;
    return true;
}

void LineDetector::getTrackInfo(TrackInfo& trackInfo) const {
    trackInfo = _trackInfo;
}

void LineDetector::getFusedVector(float& vx, float& vy) const {
    vx = _vx;
    vy = _vy;
}

// ─── Private ──────────────────────────────────────────────────────────────────

void LineDetector::normalizeVectors() {
    for (int i = 0; i < _pixy.line.numVectors; i++) {
        auto& v = _pixy.line.vectors[i];
        if (v.m_y0 < v.m_y1) {
            int tx = v.m_x0; int ty = v.m_y0;
            v.m_x0 = v.m_x1; v.m_y0 = v.m_y1;
            v.m_x1 = tx;     v.m_y1 = ty;
        }
    }
}

float LineDetector::VecLength(float x1, float y1, float x2, float y2) const {
    return sqrt((x2-x1)*(x2-x1) + (SCALE_Y(y2-y1)*SCALE_Y(y2-y1)));
}

float LineDetector::VecAngle(float x1, float y1, float x2, float y2) const {
    float dx = x2 - x1;
    float dy = SCALE_Y(y2) - SCALE_Y(y1);
    return atan2f(dy, dx) * 180.0f / PI;
}

bool LineDetector::validVector(int i) const {
    const auto& v = _pixy.line.vectors[i];
    float length = VecLength(v.m_x0, v.m_y0, v.m_x1, v.m_y1);
    if (length < MIN_VECTOR_LEN) return false;
    float angle = VecAngle(v.m_x0, v.m_y0, v.m_x1, v.m_y1);
    if (fabsf(angle) < MIN_TRACK_ANGLE) return false;   // FIX: was MIN_VECTOR_ANGLE (2°)
    return true;
}

float LineDetector::vectorWeight(int i) const {
    const auto& v = _pixy.line.vectors[i];
    float length    = VecLength(v.m_x0, v.m_y0, v.m_x1, v.m_y1);
    float proximity = v.m_y0 / (float)_pixy.frameHeight;
    return length * proximity;
}

void LineDetector::computeFusedVector() {
    _vx = 0.0f; _vy = 0.0f;
    for (int i = 0; i < _pixy.line.numVectors; i++) {
        if (!validVector(i)) continue;
        const auto& v = _pixy.line.vectors[i];
        float dx = v.m_x1 - v.m_x0;
        float dy = SCALE_Y(v.m_y1) - SCALE_Y(v.m_y0);
        float w  = vectorWeight(i);
        _vx += dx * w;
        _vy += dy * w;
    }
}

void LineDetector::normalizeFusedVector() {
    float mag = sqrt(_vx*_vx + _vy*_vy);
    if (mag > 1e-6f) { _vx /= mag; _vy /= mag; }
}

// ─── Sort helpers ─────────────────────────────────────────────────────────────

void LineDetector::sortVectors(LineVector arr[], int count) {
    for (int i = 0; i < count-1; i++)
        for (int j = 0; j < count-i-1; j++)
            if (arr[j].length < arr[j+1].length) {
                LineVector tmp = arr[j]; arr[j] = arr[j+1]; arr[j+1] = tmp;
            }
}

void LineDetector::sortLeftVectors(LineVector arr[], int count) {
    for (int i = 0; i < count-1; i++)
        for (int j = 0; j < count-i-1; j++)
            if (arr[j].x1 > arr[j+1].x1) {
                LineVector tmp = arr[j]; arr[j] = arr[j+1]; arr[j+1] = tmp;
            }
}

void LineDetector::sortRightVectors(LineVector arr[], int count) {
    for (int i = 0; i < count-1; i++)
        for (int j = 0; j < count-i-1; j++)
            if (arr[j].x1 < arr[j+1].x1) {
                LineVector tmp = arr[j]; arr[j] = arr[j+1]; arr[j+1] = tmp;
            }
}

// ─── Main scan ────────────────────────────────────────────────────────────────

TrackInfo LineDetector::Sensors_Scan(Pixy2& pixy) {
    TrackInfo info = {false, false, 0, 0, false, false};
    pixy.line.getAllFeatures();

    Serial.print("Vectors: "); Serial.println(pixy.line.numVectors);
    if (pixy.line.numVectors == 0) return info;

    LineVector lefts[10], rights[10], horz[10];
    int l_idx = 0, r_idx = 0, h_idx = 0;

    for (int i = 0; i < pixy.line.numVectors; i++) {
        LineVector v;
        v.x0 = pixy.line.vectors[i].m_x0;
        v.y0 = pixy.line.vectors[i].m_y0;
        v.x1 = pixy.line.vectors[i].m_x1;
        v.y1 = pixy.line.vectors[i].m_y1;

        float dx = v.x1 - v.x0;
        float dy = v.y1 - v.y0;
        v.length = sqrt(dx*dx + dy*dy);
        // angle from horizontal: 0° = horizontal, 90° = vertical
        v.angle  = atan2(fabsf(dy), fabsf(dx)) * 180.0f / PI;

        // FILTER 1: ignore noise
        if (v.length < 10) continue;
       

        // FILTER 2: classify as horizontal/crossing line.
        // FIX: threshold raised from 10° → MIN_TRACK_ANGLE (20°).
        // A 90° crossing produces 0° lines; raising the threshold also discards
        // slightly-tilted crossing artefacts without affecting real track lines
        // (which are typically > 45° when the camera is mounted properly).
        

        // Normalise so y0 is always the BOTTOM (larger y = closer to car)
        float intrm;
        if (v.y0 < v.y1) {
            intrm = v.x0; v.x0 = v.x1; v.x1 = intrm;
            intrm = v.y0; v.y0 = v.y1; v.y1 = intrm;
        }
        if (v.angle < MIN_TRACK_ANGLE) {
            if (h_idx < 10) horz[h_idx++] = v;
            continue;
        }
        if(v.x1>pixy.frameWidth-33 && v.x0>pixy.frameWidth-33) continue;
        if (v.x1<33 && v.x0<33) {continue;
        
        }
        if(v.y1>pixy.frameHeight-13 || v.y0<13) continue;
        
        //if(v.y1<10) continue;
         //if(v.y1<5) continue;
        // Classify left / right by the BOTTOM of the vector (where the line is
        // right now, not where it is going).
        int x_ref = (v.y0 >= v.y1) ? v.x0 : v.x1;

        if (x_ref < (pixy.frameWidth / 2)) {
            if (l_idx < 10) lefts[l_idx++] = v;
        } else {
            if (r_idx < 10) rights[r_idx++] = v;
        }
    }

    // Sort by length (longest = most reliable, put first)
    sortVectors(lefts,  l_idx);
    sortVectors(rights, r_idx);
    sortVectors(horz,   h_idx);

    // ── Intersection detection ────────────────────────────────────────────────
    // Two or more horizontal/near-horizontal lines → we are at a crossing.
     //if (/*h_idx >= 1 && (l_idx + r_idx) <= 3 && */(rights[0].y1> 165 || lefts[0].y1> 165)) info.isCrossing = true;
     //if(rights[0].y0-lefts[0].y0> 100 && lefts[0].y0!=0) info.isCrossing = true;
    // Detect intersection by checking if any vector lives entirely
    //if(onlyHasLeft == 2 || onlyHasRight == 2) info.isCrossing = true;
    //if (horz[0].x0==rights[0].x1||horz[0].x0==lefts[0].x1) info.isCrossing = true;
// in the top 25% of the frame — that's the crossing line the car
// is about to run over (or just passed)
/*for (int i = 0; i < pixy.line.numVectors; i++) {
    auto& v = pixy.line.vectors[i];
    float topThreshold = pixy.frameHeight * 0.25f;
    // Must be in top 25% AND nearly horizontal (angle < 20°)
    float dx = abs(v.m_x1 - v.m_x0);
    float dy = abs(v.m_y1 - v.m_y0);
    float angle = atan2f(dy, dx) * 180.0f / PI;
    if (v.m_y0 < topThreshold && v.m_y1 < topThreshold && angle < 20.0f) {
        info.isCrossing = true;
        break;
    }
        
        
}*/
//if(fabsf(rights[0].length-lefts[0].length)> 100 && lefts[0].length!=0&&rights[0].length!=0) info.isCrossing = true;
    // ── CONTINUITY FILTER (key fix for intersection confusion) ────────────────
    // When a crossing is active, the perpendicular track contributes extra
    // vertical-ish vectors that can be wrongly classified as the main track
    // lines. We resolve this by keeping only the vector whose HEAD (lookahead
    // tip, x1) is closest to the last reliably known position, provided it is
    // within CONTINUITY_THRESHOLD_PX pixels. If nothing is close enough we
    // fall back to memory mode (no hasLeft / hasRight), letting the steering
    // controller use _lastcurrentTargetX.
    if (info.isCrossing) {
        if (_lastHasleft && l_idx > 0) {
            int   best      = 0;
            float best_dist = fabsf(lefts[0].x1 - _lastLeftX);
            for (int i = 1; i < l_idx; i++) {
                float d = fabsf(lefts[i].x1 - _lastLeftX);
                if (d < best_dist) { best_dist = d; best = i; }
            }
            if (best_dist <= CONTINUITY_THRESHOLD_PX) {
                lefts[0] = lefts[best];   // promote best match to slot 0
                l_idx    = 1;
            } else {
                l_idx = 0;                // nothing trustworthy → memory mode
            }
        }

        if (_lastHasright && r_idx > 0) {
            int   best      = 0;
            float best_dist = fabsf(rights[0].x1 - _lastRightX);
            for (int i = 1; i < r_idx; i++) {
                float d = fabsf(rights[i].x1 - _lastRightX);
                if (d < best_dist) { best_dist = d; best = i; }
            }
            if (best_dist <= CONTINUITY_THRESHOLD_PX) {
                rights[0] = rights[best];
                r_idx     = 1;
            } else {
                r_idx = 0;
            }
        }
    }

    // ── Extract steering positions ────────────────────────────────────────────
    // Use the HEAD (x1 = lookahead tip, top of normalised vector) for steering.
    if (l_idx > 0) {
        info.hasLeft = true;
        info.leftX   = (lefts[0].y1 < lefts[0].y0) ? lefts[0].x1 : lefts[0].x0;
    }
    if (r_idx > 0) {
        info.hasRight = true;
        info.rightX   = (rights[0].y1 < rights[0].y0) ? rights[0].x1 : rights[0].x0;
    }
    if(!info.hasLeft && !info.hasRight && horz[0].length!=0){
        if((horz[0].x0<pixy.frameWidth/2)){
            info.hasLeft = true;
            info.leftX   = (horz[0].y1 < horz[0].y0) ? horz[0].x1 : horz[0].x0;
        }
        else{
            info.hasRight = true;
            info.rightX   = (horz[0].y1 < horz[0].y0) ? horz[0].x1 : horz[0].x0;
        }
        info.hasLeft = _lastHasleft;
        info.leftX = _lastLeftX;
        info.hasRight = _lastHasright;
        info.rightX = _lastRightX;
    }

    // Persist for next frame
    _lastHasleft  = info.hasLeft;
    _lastHasright = info.hasRight;
    if (info.hasLeft)  _lastLeftX  = info.leftX;
    if (info.hasRight) _lastRightX = info.rightX;
    if (info.hasLeft && !info.hasRight) onlyHasLeft++;
    if(info.hasRight && !info.hasLeft) onlyHasRight++;
    else{
        onlyHasLeft = 0;
        onlyHasRight = 0;
    }

    return info;
}
