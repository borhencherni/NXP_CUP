#pragma once
#include <Pixy2.h> 
#include "Config.h"


        struct TrackInfo {
    bool hasLeft;
    bool hasRight;
    float leftX;   // Position of left line (0-78)
    float rightX;  // Position of right line (0-78)
    bool isCrossing; // True if horizontal lines detected
    bool isFinish;   // True if finish markers detected
};

    struct LineVector {
          uint8_t x0, y0, x1, y1;
          uint8_t index;
          float length;
          float angle; 
};

class LineDetector{
    public:
        explicit LineDetector(Pixy2& pixy);
        
        bool update(); 
        //traja3 1 logique ken famma au moins 1 vecteur valide sino traja3 0 
        //+ ta3mel traitement 3la el vecteuret ken mawjoudin

        /*
        + ne7ssbou el vecteur elli nfusiw fih les vecteurs valides
        + vx entr 1 et -1 (composante horizontale: elli 3al yamin positive)
        + vy entr 1 et -1 (composante verticale: elli el foug positive)*/
        void getFusedVector(float& vx, float& vy) const;
        void getTrackInfo(TrackInfo& trackInfo) const;

    private:
        Pixy2& _pixy;

        float _vx = 0.0f;
        float _vy = 0.0f;
        TrackInfo _trackInfo;

        
        float VecLength(float x1, float y1, float x2, float y2) const;
        float VecAngle(float x1, float y1, float x2, float y2) const;
        void normalizeVectors();//negulbou les vecteurs elli nalguouhom men foug le louta
        bool validVector(int i) const;//verifier la validite d'un vecteur
        float vectorWeight(int i) const;// bech na3tou weight lkol vecteur, kol mehou a9reb  lel robot (lel louta) w kol mehou atwel kol ma el weight mte3ou a9wa
        void computeFusedVector();//les vecteurs valides elli lguinehom bech nefusohom fi vecteur we7ed w bech nadhrbouhom kol we7ed fel weight mte3ou
        void normalizeFusedVector();// el vecteur elli lguineh ki fusina les vecteur kol bech nrodouh unitair
        void sortVectors(LineVector arr[], int count);
        TrackInfo Sensors_Scan(Pixy2& pixy);


    
};
