
#ifndef Motor_h
#define Motor_h

class Motor {
  
  public:
    float rpm;
    Motor(int poles, int rpm);
    void tick();
    void set_rpm(int rpm);
    int ticks_per_phase() { return _ticks_per_phase; };
    unsigned long ticks() { return _ticks; };
    int poles; 
    
  private:
    unsigned int _ticks = 0;
    unsigned int _ticks_per_phase;
    
};

#endif
