#ifndef NAHUMTAKUM_H
#define NAHUMTAKUM_H
 class nahumtakum {

  public:
   nahumtakum(int motin1, int motin2, int motEna);
   void begin();   //must be called from  void setup()
   void run();   //must be called from  void loop()
   int getpitch(); 
   double PIDcalc(double sp, int pv);
   void tumble();
  private:
    unsigned long currentTime, previousTime, elapsedTime;
    double error, lastError;
    double input, output;
    double cumError, rateError;
    double kp = 0;
    double ki = 0; 
    double kd = 0;
    int _motin1, _motin2, _motEna;
 };
#endif 