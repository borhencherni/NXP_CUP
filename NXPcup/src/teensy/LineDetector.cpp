#include "LineDetector.h"



LineDetector::LineDetector(Pixy2& pixy) : _pixy(pixy) {}

// Public

bool LineDetector::update() {
     _pixy.line.getAllFeatures();
     if (_pixy.line.numVectors == 0) return false;
     normalizeVectors();
     computeFusedVector();
     normalizeFusedVector();
     return true;
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
     float dy = SCALE_Y(y2) - SCALE_Y(y1 - y1);
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