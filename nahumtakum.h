#ifndef NAHUMTAKUM_H
#define NAHUMTAKUM_H
 class nahumtakum {

  public:
   nahumtakum(int in1, int in2, int Ena);
   void begin();   //must be called from  void setup()
   void run();   //must be called from  void loop()
   int getpitch(); 
   double PIDcalc(double sp, int pv);
   void tumble(int kp, int ki, int kd);
  private:
    unsigned long currentTime, previousTime;
    double elapsedTime;
    double error, lastError;
    double input, output;
    double cumError, rateError;
    double _kp, _ki, _kd;
    int _in1, _in2, _Ena;
    int out;

 };
#endif 