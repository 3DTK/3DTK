#ifndef FRAME_H
#define FRAME_H

/**
 * @brief Simple frame class containing a transformation and type
 */
class Frame {
public:
  double transformation[16];
  unsigned int type;
  
  Frame() {}
  Frame(double* transformation, unsigned int type) { set(transformation, type); }
  void set(double* transformation, unsigned int type) {
    for(unsigned int i = 0; i < 16; ++i)
      this->transformation[i] = transformation[i];
    this->type = type;
  }
};

#endif //FRAME_H