#include <drivers.h>

int button()
{
  //user led
  Gpio<TGPIOB, 2, GPIO_MODE_OUT> led;   
  led = 1;  

  //user button
  Gpio<TGPIOB, 10, GPIO_MODE_IN_PULLUP> key;   

  timer.delay_ms(200);

  //wait for buttom press, slow blinking
  while ((int)key != 0)
  { 
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(200);
  }
  
  led = 1; 

  //wait for button release, fast blinking
  while ((int)key == 0)   
  { 
    led = 1; 
    timer.delay_ms(50);

    led = 0; 
    timer.delay_ms(50);
  } 

  led = 1; 

  timer.delay_ms(500); 

  return 1;
}