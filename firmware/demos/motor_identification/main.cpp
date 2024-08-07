#include <drivers.h>
#include <motor_identification.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    motor_identification();

   
    while (true) 
    {
       
    }

    return 0;
}
  