#include "LineDetector.h"
#include "usb_serial.h"



LineDetector::LineDetector(Pixy2& pixy) : _pixy(pixy) {}

// Public

bool LineDetector::update() {
     //_pixy.line.getAllFeatures();
      _trackInfo = Sensors_Scan(_pixy);
     if (!_trackInfo.hasRight && !_trackInfo.hasLeft) return false;
     /*normalizeVectors();
     computeFusedVector();
     normalizeFusedVector();*/
     return true;
     
}


void LineDetector::getTrackInfo(TrackInfo& trackInfo) const {
   trackInfo = _trackInfo;
}
void LineDetector::getFusedVector(float& vx, float& vy) const {
     vx = _vx;
     vy = _vy;
}

// Private

void LineDetector::normalizeVectors() {//negulbou les vecteurs elli nalguouhom men foug le louta
     for (int i = 0; i < _pixy.line.numVectors; i++) {
          auto& v = _pixy.line.vectors[i];
          if (v.m_y0 < v.m_y1) {
            int tx = v.m_x0;
            int ty = v.m_y0;

            v.m_x0 = v.m_x1;  v.m_y0 = v.m_y1;
            v.m_x1 = tx;       v.m_y1 = ty;
          }
     }}



float LineDetector::VecLength(float x1, float y1, float x2, float y2) const {
     return sqrt((x2 - x1) * (x2 - x1) + (SCALE_Y(y2 - y1) * SCALE_Y(y2 - y1)));
}


float LineDetector::VecAngle (float x1, float y1, float x2, float y2) const {
     float dx = x2 - x1;
     float dy = SCALE_Y(y2) - SCALE_Y(y1);
     return atan2f(dy, dx) * 180.0f / PI;
}



bool LineDetector::validVector(int i) const {//verifier la validite d'un vecteur
     const auto& v = _pixy.line.vectors[i];
     

     float length = VecLength(v.m_x0, v.m_y0, v.m_x1, v.m_y1);

     if (length < MIN_VECTOR_LEN) return false;

     float angle = VecAngle(v.m_x0, v.m_y0, v.m_x1, v.m_y1);

     if(fabsf(angle) < MIN_VECTOR_ANGLE) return false;

     return true;
}

float LineDetector::vectorWeight(int i) const {// bech na3tou weight lkol vecteur, kol mehou a9reb  lel robot (lel louta) w kol mehou atwel kol ma el weight mte3ou a9wa
     const auto& v = _pixy.line.vectors[i];
     
     float length = VecLength(v.m_x0, v.m_y0, v.m_x1, v.m_y1);
     float proximity = v.m_y0 / (float)_pixy.frameHeight;
     return length * proximity;
}


void LineDetector::computeFusedVector() {//les vecteurs valides elli lguinehom bech nefusohom fi vecteur we7ed w bech nadhrbouhom kol we7ed fel weight mte3ou
     _vx = 0.0f;
     _vy = 0.0f;
     for (int i = 0; i < _pixy.line.numVectors; i++) {
          if (!validVector(i)) continue;
          const auto& v = _pixy.line.vectors[i];
          float dx = v.m_x1 - v.m_x0;
          float dy = SCALE_Y(v.m_y1) - SCALE_Y(v.m_y0);
          float w = vectorWeight(i);
          _vx += dx * w;
          _vy += dy * w;
     }
}

void LineDetector::normalizeFusedVector() {// el vecteur elli lguineh ki fusina les vecteur kol bech nrodouh unitair
     float mag = sqrt(_vx * _vx + _vy * _vy);

     if (mag > 1e-6f){
          _vx /= mag;
          _vy /= mag;
     }
 
}

TrackInfo LineDetector::Sensors_Scan(Pixy2& pixy) {
    TrackInfo info = {false, false, 0, 0, false, false};
    pixy.line.getAllFeatures();
    Serial.print("Number of Vectors: ");
    Serial.println(pixy.line.numVectors);
    if (pixy.line.numVectors == 0) return info;

    LineVector lefts[10], rights[10], horz[10];
    
    int l_idx = 0, r_idx = 0, h_idx=0;
    int horizontalCount = 0;

    for (int i = 0; i < pixy.line.numVectors; i++) {
        // Convert Pixy internal struct to our local struct
        LineVector v;
        v.x0 = pixy.line.vectors[i].m_x0;
        v.y0 = pixy.line.vectors[i].m_y0;
        v.x1 = pixy.line.vectors[i].m_x1;
        v.y1 = pixy.line.vectors[i].m_y1;
        
        float dx = v.x1 - v.x0;
        float dy = v.y1 - v.y0;
        v.length = sqrt(dx*dx + dy*dy);
        v.angle = atan2(abs(dy), abs(dx)) * 180.0 / PI;

        // FILTER 1: IGNORE NOISE (Tiny lines)
        if (v.length < 10) continue;

        // FILTER 2: DETECT INTERSECTION / FINISH (Horizontal Lines)
        if (v.angle < 10.0) { 
            
            horz[h_idx++] = v;
            continue; // DO NOT use this for steering
        }
        float intrm;
        if(v.y0<v.y1){
                intrm = v.x0;
                v.x0 = v.x1;
                v.x1 = intrm;
                intrm = v.y0;
                v.y0 = v.y1;
                v.y1 = intrm;
        }
        
        // CLASSIFY LEFT vs RIGHT
        // We use the "Bottom" of the vector (y max) as reference
        int x_ref = (v.y0 > v.y1) ? v.x0 : v.x1; 

        if (x_ref < (pixy.frameWidth / 2)) {
            if (l_idx < 10) lefts[l_idx++] = v; // Safety Check < 10
        } else {
            if (r_idx < 10) rights[r_idx++] = v; // Safety Check < 10
        }
    }
    sortVectors(lefts, l_idx);
    sortVectors(rights, r_idx);
    sortVectors(horz, h_idx);

    // Sort to find the "Main" lines (longest)
    if(lefts[0].x0>((pixy.frameWidth/2)-82)){
         sortLeftVectors(lefts, l_idx);
    }
    if(rights[0].x0<((pixy.frameWidth/2)+82)){
         sortLeftVectors(rights, l_idx);
    }
    

    if (l_idx > 0) {
        info.hasLeft = true;
        // Use the "Head" (Top) of the vector for looking ahead
        info.leftX = (lefts[0].y1 < lefts[0].y0) ? lefts[0].x1 : lefts[0].x0;
    }
    
    if (r_idx > 0) {
        info.hasRight = true;
        info.rightX = (rights[0].y1 < rights[0].y0) ? rights[0].x1 : rights[0].x0;
    }
    
    /*if(_lastHasleft && !info.hasLeft) {while(true){_speedCtrl.runMotors(0, 0);}}
    if(_lastHasright && !info.hasRight)  {while(true){_speedCtrl.runMotors(0, 0);}}*/
   /* if(info.hasLeft && info.hasRight){
        Serial.print("there is Voectors");

    }*/
    if(info.hasRight && !info.hasLeft) {while(true){_speedCtrl.runMotors(0, 0);}}

    // LOGIC: Finish Line = Multiple Horizontal Lines
    if (h_idx>= 2)  {
          info.isCrossing = true;
            /*if ((horz[0].x0 < info.rightX) && (horz[0].x1 < info.rightX)&&(horz[0].x0 > info.leftX) && (horz[0].x1 > info.leftX)  ){
                  if ((horz[1].x0 < info.rightX) && (horz[1].x1 < info.rightX)&&(horz[1].x0 > info.leftX) && (horz[1].x1 > info.leftX)){
                                         info.isFinish = true;}}
            else info.isCrossing = true;}
          else if(h_idx> 2)   {info.isCrossing = true;}*/
                                         }
     _lastHasleft = info.hasLeft;
     _lastHasright = info.hasRight;
     _lastLeftX = info.leftX;
     _lastRightX = info.rightX;
     return info;
}



void LineDetector::sortLeftVectors(LineVector arr[], int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (arr[j].x1> arr[j + 1].x1) {
                LineVector temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void LineDetector::sortRightVectors(LineVector arr[], int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (arr[j].x1 < arr[j + 1].x1) {
                LineVector temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void LineDetector::sortVectors(LineVector arr[], int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (arr[j].length < arr[j + 1].length) {
                LineVector temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}